/*! ----------------------------------------------------------------------------
*  @file    ss_TOF_bidirectionnel_main.c
*  @brief   SS-TWR bidirectionnel : chaque carte est a la fois initiateur et repondeur.
*
*           - Toutes les POLL_KNOWN_PERIOD_MS (1 s) la carte poll les cartes deja connues.
*           - Toutes les DISCOVER_PERIOD_MS (10 s) elle envoie un poll en diffusion pour decouvrir
*             de nouvelles cartes.
*           - Le reste du temps le recepteur du DW1000 reste allume. Des qu'une trame arrive,
*             l'IRQ du DW1000 declenche dwt_isr() -> rx_ok_cb() qui lit la trame, repond au
*             poll (emission differee) ou enregistre la reponse recue. Les messages UART sont
*             ensuite affiches par la tache (jamais dans l'interruption).
*           - Apres chaque mesure, la distance (en cm) est renvoyee a la carte qui vient de repondre
*             dans une trame FUNC_DISTANCE (0xE2) : source = carte qui a mesure, destinataire = carte
*             mesuree. Chaque carte connait ainsi la distance qui la separe de ses voisines.
*
*           Chaine : DW1000 IRQ -> GPIOTE (main.c, vInterruptHandler) -> dwt_isr() -> callbacks.
*
*           Base sur ss_init_main.c / ss_resp_main.c, Copyright Decawave Ltd, Dublin, Ireland.
*/
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "nordic_common.h"
#include "boards.h"
#include "nrf.h"
#include "nrf_gpio.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "port_platform.h"
#include "UART.h"

/* ------------------------------------------------------------------------------------------- */
/* Gardes : valeurs par defaut si le projet ne les definit pas deja                            */
/* ------------------------------------------------------------------------------------------- */
#ifndef DW1000_IRQ
#define DW1000_IRQ 19
#endif
#ifndef TX_ANT_DLY
#define TX_ANT_DLY 16300
#endif
#ifndef DWT_INT_SFDT
#define DWT_INT_SFDT 0x04000000      /* RXSFDTO : bit 26 de SYS_MASK */
#endif

/* ------------------------------------------------------------------------------------------- */
/* Parametres                                                                                  */
/* ------------------------------------------------------------------------------------------- */
#define POLL_KNOWN_PERIOD_MS   1000     /* poll des cartes connues : toutes les secondes */
#define DISCOVER_PERIOD_MS    10000     /* recherche de nouvelles cartes : toutes les 10 s */
#define RESP_WAIT_MS             10     /* attente de la reponse (la reponse arrive en ~1,5 ms) */

#define TARGET_ALL       0xFFFF      /* adresse de diffusion */
#define FUNC_POLL        0xE0
#define FUNC_RESP        0xE1
#define FUNC_DISTANCE    0xE2

/* Delai poll recu -> reponse emise (valeur de ss_resp_main.c pour le nRF) */
#define POLL_RX_TO_RESP_TX_DLY_UUS 1100

#define UUS_TO_DWT_TIME  65536
#define SPEED_OF_LIGHT   299702547

/* ------------------------------------------------------------------------------------------- */
/* Trames                                                                                      */
/* ------------------------------------------------------------------------------------------- */
static const uint8 rx_hdr[5] = {0x41, 0x88, 0, 0xCA, 0xDE};   /* FC, SN (=0), PAN ID */

static uint8 tx_poll_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 0, 0, 0, 0, FUNC_POLL, 0, 0};
static uint8 tx_resp_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 0, 0, 0, 0, FUNC_RESP, 0,0,0,0, 0,0,0,0, 0, 0};

/* Trame de distance : source = carte qui a mesure, destinataire = carte mesuree (celle qui a envoye la reponse).
   Octets 10-13 = distance en cm (int32, poids faible d'abord), 14-15 = CRC. */
static uint8 tx_dist_msg[] = {0x41, 0x88, 0, 0xCA, 0xDE, 0, 0, 0, 0, FUNC_DISTANCE,
                              0, 0, 0, 0,
                              0, 0};

#define ALL_MSG_SN_IDX          2
#define RESP_MSG_POLL_RX_TS_IDX 10
#define RESP_MSG_RESP_TX_TS_IDX 14
#define RESP_MSG_TS_LEN         4
#define DIST_MSG_VAL_IDX        10

#define RX_BUF_LEN 20                /* la plus longue trame (reponse) : 20 octets avec le CRC */
static uint8 rx_buffer[RX_BUF_LEN];

/* ------------------------------------------------------------------------------------------- */
/* Etat partage entre l'interruption et la tache                                               */
/* ------------------------------------------------------------------------------------------- */
static uint16_t my_id;                          /* 16 bits bas de NRF_FICR->DEVICEID[0] */
static uint8    frame_seq_nb = 0;
static volatile int tx_count = 0;
static volatile int rx_count = 0;

static volatile uint8    last_tx_func     = 0;  /* FUNC_POLL, FUNC_RESP ou FUNC_DISTANCE : dernier type emis */
static volatile uint32   poll_tx_ts_saved = 0;  /* horodatage d'emission de MON dernier poll */
static volatile bool     awaiting_resp    = false;
static volatile uint16_t poll_dest        = TARGET_ALL;

static double tof;
static double distance;

/* File d'evenements : l'interruption ecrit, la tache lit (un seul producteur, un seul consommateur) */
#define EVT_POLL_RX     1
#define EVT_RESP_RX     2
#define EVT_DISTANCE_RX 3

typedef struct {
  uint8    type;
  uint16_t src;
  uint16_t dest;
  bool     resp_sent;                           /* EVT_POLL_RX : la reponse a-t-elle pu partir ? */
  uint32   poll_tx_ts, resp_rx_ts;              /* EVT_RESP_RX : horodatages locaux */
  uint32   poll_rx_ts, resp_tx_ts;              /* EVT_RESP_RX : horodatages du repondeur */
  int32    carrier;
  int32    distance_cm;                         /* EVT_DISTANCE_RX : distance recue */
} evt_t;

#define EVT_N 16
static evt_t evt_q[EVT_N];
static volatile uint8 evt_w = 0;
static volatile uint8 evt_r = 0;

static evt_t *evt_slot(void)
{
  uint8 nxt = (uint8)((evt_w + 1) % EVT_N);
  if (nxt == evt_r) return NULL;                /* file pleine : evenement perdu */
  return &evt_q[evt_w];
}

static void evt_commit(void)
{
  __DMB();                                      /* donnees ecrites avant l'index */
  evt_w = (uint8)((evt_w + 1) % EVT_N);
}

/* Liste des cartes connues (cote tache uniquement) */
#define MAX_TAGS 50
static uint16_t tag_ids[MAX_TAGS];
static float    tag_distances[MAX_TAGS];
static uint8_t  tag_count = 0;

/* ------------------------------------------------------------------------------------------- */
/* Utilitaires horodatages                                                                     */
/* ------------------------------------------------------------------------------------------- */
static uint64 get_rx_timestamp_u64(void)
{
  uint8 ts_tab[5];
  uint64 ts = 0;
  int i;
  dwt_readrxtimestamp(ts_tab);
  for (i = 4; i >= 0; i--)
  {
    ts <<= 8;
    ts |= ts_tab[i];
  }
  return ts;
}

static void resp_msg_set_ts(uint8 *ts_field, const uint64 ts)
{
  int i;
  for (i = 0; i < RESP_MSG_TS_LEN; i++)
  {
    ts_field[i] = (ts >> (i * 8)) & 0xFF;
  }
}

static void resp_msg_get_ts(const uint8 *ts_field, uint32 *ts)
{
  int i;
  *ts = 0;
  for (i = 0; i < RESP_MSG_TS_LEN; i++)
  {
    *ts += (uint32)ts_field[i] << (i * 8);
  }
}

/* ------------------------------------------------------------------------------------------- */
/* Cote interruption (callbacks appeles par dwt_isr) : pas de uart_printf, pas d'attente       */
/* ------------------------------------------------------------------------------------------- */
static void rx_restart(void)
{
  dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

/* Reponse a un poll : emission differee de POLL_RX_TO_RESP_TX_DLY_UUS apres la reception */
static bool send_RESP(uint16_t dest)
{
  uint32 resp_tx_time;
  uint64 poll_rx_ts, resp_tx_ts;

  poll_rx_ts = get_rx_timestamp_u64();

  resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;
  dwt_setdelayedtrxtime(resp_tx_time);
  resp_tx_ts = (((uint64)(resp_tx_time & 0xFFFFFFFEUL)) << 8) + TX_ANT_DLY;

  tx_resp_msg[5] = dest & 0xFF;     tx_resp_msg[6] = dest >> 8;       /* destination = emetteur du poll */
  tx_resp_msg[7] = my_id & 0xFF;    tx_resp_msg[8] = my_id >> 8;      /* source = moi */
  resp_msg_set_ts(&tx_resp_msg[RESP_MSG_POLL_RX_TS_IDX], poll_rx_ts);
  resp_msg_set_ts(&tx_resp_msg[RESP_MSG_RESP_TX_TS_IDX], resp_tx_ts);
  tx_resp_msg[ALL_MSG_SN_IDX] = frame_seq_nb++;

  last_tx_func = FUNC_RESP;
  dwt_writetxdata(sizeof(tx_resp_msg), tx_resp_msg, 0);
  dwt_writetxfctrl(sizeof(tx_resp_msg), 0, 1);

  return (dwt_starttx(DWT_START_TX_DELAYED) == DWT_SUCCESS);
}

/* Fin d'emission (poll, reponse ou distance) */
static void tx_ok_cb(const dwt_cb_data_t *cb_data)
{
  (void)cb_data;
  if (last_tx_func == FUNC_POLL)
  {
    /* Horodatage d'emission de mon poll, sauvegarde ici car une reponse a un autre poll
       ecraserait le registre. Le recepteur est deja arme par DWT_RESPONSE_EXPECTED. */
    poll_tx_ts_saved = dwt_readtxtimestamplo32();
  }
  else
  {
    rx_restart();                               /* reponse ou distance partie : retour a l'ecoute */
  }
}

/* Trame recue avec CRC correct */
static void rx_ok_cb(const dwt_cb_data_t *cb_data)
{
  bool tx_started = false;
  uint32 len = cb_data->datalength;

  if (len <= RX_BUF_LEN)
  {
    dwt_readrxdata(rx_buffer, len, 0);
    rx_buffer[ALL_MSG_SN_IDX] = 0;              /* numero de sequence ignore pour la comparaison */

    if (memcmp(rx_buffer, rx_hdr, 5) == 0)
    {
      uint16_t rx_dest = rx_buffer[5] | (rx_buffer[6] << 8);
      uint16_t rx_src  = rx_buffer[7] | (rx_buffer[8] << 8);
      uint8    rx_func = rx_buffer[9];

      /* Poll recu, en diffusion ou pour moi : je reponds tout de suite */
      if (rx_func == FUNC_POLL && (rx_dest == TARGET_ALL || rx_dest == my_id) && rx_src != my_id)
      {
        tx_started = send_RESP(rx_src);

        evt_t *e = evt_slot();
        if (e)
        {
          e->type      = EVT_POLL_RX;
          e->src       = rx_src;
          e->dest      = rx_dest;
          e->resp_sent = tx_started;
          evt_commit();
        }
      }
      /* Reponse a mon poll : on garde les horodatages, la tache calcule la distance */
      else if (rx_func == FUNC_RESP && rx_dest == my_id && awaiting_resp &&
               (poll_dest == TARGET_ALL || poll_dest == rx_src))
      {
        evt_t *e = evt_slot();
        if (e)
        {
          e->type       = EVT_RESP_RX;
          e->src        = rx_src;
          e->dest       = rx_dest;
          e->poll_tx_ts = poll_tx_ts_saved;
          e->resp_rx_ts = dwt_readrxtimestamplo32();
          e->carrier    = dwt_readcarrierintegrator();
          resp_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &e->poll_rx_ts);
          resp_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &e->resp_tx_ts);
          evt_commit();
          awaiting_resp = false;
        }
      }
      /* Distance que la carte rx_src vient de mesurer entre elle et moi */
      else if (rx_func == FUNC_DISTANCE)
      {
        evt_t *e = evt_slot();
        if (e)
        {
          e->type        = EVT_DISTANCE_RX;
          e->src         = rx_src;
          e->dest        = rx_dest;
          e->distance_cm = (int32)((uint32)rx_buffer[DIST_MSG_VAL_IDX]
                                 | ((uint32)rx_buffer[DIST_MSG_VAL_IDX + 1] << 8)
                                 | ((uint32)rx_buffer[DIST_MSG_VAL_IDX + 2] << 16)
                                 | ((uint32)rx_buffer[DIST_MSG_VAL_IDX + 3] << 24));
          evt_commit();
        }
      }
    }
  }

  if (!tx_started)
  {
    rx_restart();     /* sinon tx_ok_cb relancera l'ecoute apres l'emission de la reponse */
  }
}

/* Timeout : ne devrait pas arriver (timeout de trame a 0), on reste a l'ecoute */
static void rx_to_cb(const dwt_cb_data_t *cb_data)
{
  (void)cb_data;
  rx_restart();
}

/* Erreur de reception (PHR, CRC, Reed-Solomon, timeout SFD...) : reset du recepteur et reecoute */
static void rx_err_cb(const dwt_cb_data_t *cb_data)
{
  (void)cb_data;
  dwt_rxreset();
  rx_restart();
}

/* ------------------------------------------------------------------------------------------- */
/* Cote tache                                                                                  */
/* ------------------------------------------------------------------------------------------- */
/* decamutexon()/decamutexoff() du port Decawave ne font rien ("currently do nothing").
   On masque donc nous-memes l'interruption GPIOTE (celle de la broche IRQ du DW1000) pendant
   les acces SPI de la tache : un front montant arrive pendant ce temps reste en attente dans le
   NVIC et l'interruption est executee des qu'on la reactive. */
static uint32_t dw_irq_off(void)
{
  uint32_t was_enabled = NVIC->ISER[GPIOTE_IRQn >> 5] & (1UL << (GPIOTE_IRQn & 0x1F));
  NVIC_DisableIRQ(GPIOTE_IRQn);
  __DSB();
  __ISB();
  return was_enabled;
}

static void dw_irq_on(uint32_t was_enabled)
{
  if (was_enabled) NVIC_EnableIRQ(GPIOTE_IRQn);
}

/* Envoi de la distance mesuree (cote tache) a la carte qui vient de repondre.
   Pas de reponse attendue : emission immediate.
   dest : carte mesuree (celle qui a envoye la reponse)
   cm   : distance en centimetres */
static void send_DISTANCE(uint16_t dest, int32 cm)
{
  uint32_t en = dw_irq_off();

  dwt_forcetrxoff();                            /* arrete l'ecoute en cours */

  tx_dist_msg[5] = dest & 0xFF;     tx_dist_msg[6] = dest >> 8;       /* destination = carte mesuree */
  tx_dist_msg[7] = my_id & 0xFF;    tx_dist_msg[8] = my_id >> 8;      /* source = carte qui a mesure */
  tx_dist_msg[ALL_MSG_SN_IDX] = frame_seq_nb++;
  for (int i = 0; i < 4; i++)
  {
    tx_dist_msg[DIST_MSG_VAL_IDX + i] = ((uint32)cm >> (8 * i)) & 0xFF;
  }

  last_tx_func = FUNC_DISTANCE;
  dwt_writetxdata(sizeof(tx_dist_msg), tx_dist_msg, 0);
  dwt_writetxfctrl(sizeof(tx_dist_msg), 0, 0);  /* 0 : trame de donnees, pas de ranging */

  if (dwt_starttx(DWT_START_TX_IMMEDIATE) != DWT_SUCCESS)
  {
    rx_restart();
  }

  dw_irq_on(en);
}

static int tag_index(uint16_t id)
{
  for (int i = 0; i < tag_count; i++)
  {
    if (tag_ids[i] == id) return i;
  }
  return -1;
}

static void add_tag_in_my_list(uint16_t id)
{
  if (id == my_id || id == TARGET_ALL) return;
  if (tag_index(id) >= 0) return;

  if (tag_count < MAX_TAGS)
  {
    tag_ids[tag_count] = id;
    tag_distances[tag_count] = 0.0f;
    tag_count++;
    // uart_printf("New tag detected: %04X \r\n", (unsigned)id);
  }
  // else
  // {
  //   uart_printf("Tag list full! \r\n");
  // }
}

/* Calcule la distance (m) a partir des horodatages */
static double compute_distance(const evt_t *e)
{
  float clockOffsetRatio = e->carrier * (FREQ_OFFSET_MULTIPLIER * HERTZ_TO_PPM_MULTIPLIER_CHAN_5 / 1.0e6);
  int32 rtd_init = e->resp_rx_ts - e->poll_tx_ts;
  int32 rtd_resp = e->resp_tx_ts - e->poll_rx_ts;

  tof = ((rtd_init - rtd_resp * (1.0f - clockOffsetRatio)) / 2.0f) * DWT_TIME_UNITS;
  distance = tof * SPEED_OF_LIGHT;
  if (distance < 0.0) return 0.0;               /* evite les valeurs negatives */

  return distance;
}

/* Traite (UART, envoi des distances) ce que l'interruption a recu.
   Format des lignes : carte_qui_mesure,carte_mesuree,distance_m */
static void handle_events(void)
{
  while (evt_r != evt_w)
  {
    evt_t e = evt_q[evt_r];
    evt_r = (uint8)((evt_r + 1) % EVT_N);

    if (e.type == EVT_POLL_RX)
    {
      // uart_printf("Reception POLL from %04X \r\n", (unsigned)e.src);
      // if (e.resp_sent)
      //   uart_printf("Send RESP from %04X to %04X \r\n", (unsigned)my_id, (unsigned)e.src);
      // else
      //   uart_printf("RESP trop tardive\r\n");
      add_tag_in_my_list(e.src);
    }
    else if (e.type == EVT_RESP_RX)
    {
      double d = compute_distance(&e);
      int32  cm = (int32)(d * 100.0 + 0.5);
      int idx;

      rx_count++;
      add_tag_in_my_list(e.src);
      idx = tag_index(e.src);
      if (idx >= 0) tag_distances[idx] = (float)d;

      // uart_printf("Reception RESP from %04X \r\n", (unsigned)e.src);
      uart_printf("%04X,%04X,%.2f \r\n", (unsigned)my_id, (unsigned)e.src, d);

      send_DISTANCE(e.src, cm);                 /* renvoie la mesure a la carte qui a repondu */
    }
    else if (e.type == EVT_DISTANCE_RX)
    {
      /* e.src a mesure la distance entre elle et moi (e.dest = my_id) */
      add_tag_in_my_list(e.src);
      uart_printf("%04X,%04X,%.2f \r\n", (unsigned)e.src, (unsigned)e.dest, e.distance_cm / 100.0);
    }
  }
}

/* Filet de securite : si la broche IRQ est restee a 1 (front montant perdu pendant que
   l'interruption etait masquee), on traite l'evenement a la main. */
static void irq_safety_net(void)
{
  if (nrf_gpio_pin_read(DW1000_IRQ))
  {
    uint32_t en = dw_irq_off();
    dwt_isr();
    dw_irq_on(en);
  }
}

/* Attend ms millisecondes en traitant les evenements au fil de l'eau (pas de blocage du recepteur) */
static void wait_and_handle(uint32_t ms)
{
  TickType_t start = xTaskGetTickCount();
  TickType_t dur   = pdMS_TO_TICKS(ms);

  while ((TickType_t)(xTaskGetTickCount() - start) < dur)
  {
    handle_events();
    irq_safety_net();
    vTaskDelay(1);
  }
  handle_events();
}

/* Envoi d'un poll (interruption DW1000 masquee pendant l'acces SPI) */
static void send_POLL(uint16_t dest)
{
  uint32_t en = dw_irq_off();

  dwt_forcetrxoff();                            /* arrete l'ecoute en cours */

  tx_poll_msg[5] = dest & 0xFF;     tx_poll_msg[6] = dest >> 8;       /* destination */
  tx_poll_msg[7] = my_id & 0xFF;    tx_poll_msg[8] = my_id >> 8;      /* source */
  tx_poll_msg[ALL_MSG_SN_IDX] = frame_seq_nb++;

  last_tx_func  = FUNC_POLL;
  poll_dest     = dest;
  awaiting_resp = true;

  dwt_writetxdata(sizeof(tx_poll_msg), tx_poll_msg, 0);
  dwt_writetxfctrl(sizeof(tx_poll_msg), 0, 1);

  /* Le recepteur s'allume tout seul apres dwt_setrxaftertxdelay() (regle dans main.c) */
  if (dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED) != DWT_SUCCESS)
  {
    awaiting_resp = false;
    rx_restart();
  }
  tx_count++;

  dw_irq_on(en);
}

int ss_init_run(uint16_t target_id)
{
  send_POLL(target_id);
  // uart_printf("Send POLL from %04X to %04X \r\n", (unsigned)my_id, (unsigned)target_id);

  wait_and_handle(RESP_WAIT_MS);                /* la reponse arrive par interruption */

  if (awaiting_resp)
  {
    awaiting_resp = false;
    // if (target_id != TARGET_ALL)
    //   uart_printf("Pas de reponse de %04X \r\n", (unsigned)target_id);
  }
  return 1;
}

/* Poll direct de chaque carte connue (pas de diffusion) */
static void poll_known_tags(void)
{
  uint8_t n = tag_count;
  for (uint8_t i = 0; i < n; i++)
  {
    ss_init_run(tag_ids[i]);
  }
}

static bool is_due(TickType_t t)
{
  return (int32_t)(xTaskGetTickCount() - t) >= 0;
}

/**@brief Tache : configure les interruptions du DW1000, puis contacte les autres cartes.
*/
void ss_initiator_task_function (void * pvParameter)
{
  UNUSED_PARAMETER(pvParameter);

  my_id = (uint16_t)(NRF_FICR->DEVICEID[0] & 0xFFFF);
  uart_printf("Mon id : %04X\r\n", (unsigned)my_id);

  /* Interruptions DW1000 : le masque (SYS_MASK, 0x0E) est vide apres la mise sous tension.
     TFRS = emission terminee, RFCG = trame correcte, RFTO = timeout de trame,
     RPHE / RFCE / RFSL / SFDT = erreurs de reception (sans elles le recepteur resterait
     eteint apres une erreur, y compris le timeout SFD de la configuration). */
  dwt_setcallbacks(tx_ok_cb, rx_ok_cb, rx_to_cb, rx_err_cb);
  dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | DWT_INT_RFTO |
                   DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFSL | DWT_INT_SFDT, 1);
  uart_printf("SYS_MASK = %08lX\r\n", (unsigned long)dwt_read32bitreg(SYS_MASK_ID));

  dwt_setleds(DWT_LEDS_ENABLE);

  {
    uint32_t en = dw_irq_off();
    rx_restart();                               /* ecoute permanente (timeout de trame = 0 dans main.c) */
    dw_irq_on(en);
  }

  /* Decalage au demarrage propre a chaque carte, pour que deux cartes n'emettent pas en meme temps */
  wait_and_handle(500 + (my_id % 500));

  TickType_t next_disc = xTaskGetTickCount();   /* decouverte des le depart */
  TickType_t next_poll = next_disc;

  while (true)
  {
    if (is_due(next_disc))
    {
      /* Toutes les 10 s : diffusion pour trouver de nouvelles cartes (remplace le poll de cette seconde) */
      TickType_t t0 = xTaskGetTickCount();
      ss_init_run(TARGET_ALL);
      next_disc = t0 + pdMS_TO_TICKS(DISCOVER_PERIOD_MS);
      next_poll = t0 + pdMS_TO_TICKS(POLL_KNOWN_PERIOD_MS);
    }
    else if (is_due(next_poll))
    {
      /* Sinon, toutes les secondes : poll des cartes deja connues */
      TickType_t t0 = xTaskGetTickCount();
      poll_known_tags();
      next_poll = t0 + pdMS_TO_TICKS(POLL_KNOWN_PERIOD_MS);
    }

    wait_and_handle(10);                        /* ecoute, reponses et affichage entre deux echeances */
  }
}

/*****************************************************************************************************************************************************
* NOTES:
*
* 1. Trames (IEEE 802.15.4, adresses sur 16 bits) : octets 0-1 frame control (0x8841), 2 numero de sequence, 3-4 PAN ID (0xDECA),
*    5-6 adresse destination, 7-8 adresse source, 9 code fonction (0xE0 = poll, 0xE1 = reponse, 0xE2 = distance).
*    Reponse : deux horodatages de 4 octets (reception du poll, emission de la reponse, octets 10-13 et 14-17), puis le CRC.
*    Distance : octets 10-13 distance en cm (int32, poids faible d'abord), puis le CRC (2 octets) ajoute par le DW1000, soit 16 octets.
*    Source = carte qui a mesure, destinataire = carte mesuree : pas de champ supplementaire.
* 2. Adresse = 16 bits de poids faible de NRF_FICR->DEVICEID[0] : unique par carte en pratique, aucune valeur codee en dur.
*    0xFFFF est l'adresse de diffusion (poll de decouverte uniquement).
* 3. Reception par interruption : le masque SYS_MASK doit etre ecrit (dwt_setinterrupt), sinon la broche IRQ ne se leve jamais. Les bits de
*    SYS_STATUS sont acquittes par dwt_isr(). Apres une bonne trame en tampon simple (RXAUTR = 0) le recepteur s'arrete : rx_restart() le relance.
*    Apres un timeout de trame, il ne se reactive jamais seul (manuel DW1000, registre 0x0C) : d'ou le timeout a 0 dans main.c.
* 4. Si plusieurs cartes repondent en meme temps a un poll en diffusion, leurs reponses se superposent : seule la premiere reponse
*    exploitable est prise, les autres cartes sont decouvertes quand elles contactent cette carte a leur tour.
* 5. Les callbacks s'executent dans l'interruption : ils n'appellent ni uart_printf ni fonction FreeRTOS. Les messages sont affiches par la tache,
*    au plus une periode du tick FreeRTOS apres l'interruption.
* 6. La reponse doit etre emise POLL_RX_TO_RESP_TX_DLY_UUS apres le poll ; si dwt_starttx() echoue, la reponse ne part pas
*    (reactiver le message "RESP trop tardive" dans handle_events) : augmenter alors la valeur.
* 7. Le DW1000 n'ecoute pas avant d'emettre : deux emissions simultanees se perdent, sans accuse de reception. Les trames de distance
*    ne sont pas retransmises.
*
****************************************************************************************************************************************************/
