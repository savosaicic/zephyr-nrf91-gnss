#include <zephyr/kernel.h>
#include <modem/nrf_modem_lib.h>
#include <nrf_modem_gnss.h>
#include <zephyr/logging/log.h>
#include <modem/lte_lc.h>
#include <nrf_modem_at.h>
#include <date_time.h>
#include <stdint.h>

LOG_MODULE_REGISTER(nrf91_gnss);

static K_SEM_DEFINE(lte_connected, 0, 1);
static K_SEM_DEFINE(time_synced, 0, 1);
static K_SEM_DEFINE(agnss_req_ready, 0, 1);

static struct nrf_modem_gnss_agnss_data_frame gnss_agnss_req;
static struct nrf_modem_gnss_pvt_data_frame   pvt_data;

static int64_t gnss_start_time;
static bool    first_fix = false;

static void print_fix_data(struct nrf_modem_gnss_pvt_data_frame *pvt)
{
  LOG_INF("Latitude:       %.06f", pvt->latitude);
  LOG_INF("Longitude:      %.06f", pvt->longitude);
  LOG_INF("Altitude:       %.01f m", (double)pvt->altitude);
  LOG_INF("Time (UTC):     %02u:%02u:%02u.%03u", pvt->datetime.hour,
          pvt->datetime.minute, pvt->datetime.seconds, pvt->datetime.ms);
}

static void  gnss_event_handler(int evt)
{
  int err;

  switch (evt) {
  case NRF_MODEM_GNSS_EVT_AGNSS_REQ:
    err = nrf_modem_gnss_read(&gnss_agnss_req, sizeof(gnss_agnss_req),
                              NRF_MODEM_GNSS_DATA_AGNSS_REQ);
    if (err) {
      LOG_ERR("Failed to read A-GNSS request: %d", err);
      break;
    }
    LOG_INF("A-GNSS request from modem: data_flags=0x%08X",
            gnss_agnss_req.data_flags);
    k_sem_give(&agnss_req_ready);
    break;

  case NRF_MODEM_GNSS_EVT_PVT:
    err = nrf_modem_gnss_read(&pvt_data, sizeof(pvt_data),
                              NRF_MODEM_GNSS_DATA_PVT);
    if (err) {
      LOG_ERR("nrf_modem_gnss_read failed, err %d", err);
      return;
    }

    int num_satellites = 0;
    for (int i = 0; i < NRF_MODEM_GNSS_MAX_SATELLITES; i++) {
      if (pvt_data.sv[i].signal != 0) {
        LOG_INF("sv: %d, cn0: %d, signal: %d", pvt_data.sv[i].sv,
                pvt_data.sv[i].cn0, pvt_data.sv[i].signal);
        num_satellites++;
      }
    }
    LOG_INF("Searching... satellites in view: %d", num_satellites);

    if (pvt_data.flags & NRF_MODEM_GNSS_PVT_FLAG_FIX_VALID) {
      print_fix_data(&pvt_data);
      if (!first_fix) {
        LOG_INF("TTFF: %2.1lld s", (k_uptime_get() - gnss_start_time) / 1000);
        first_fix = true;
      }
    }

    if (pvt_data.flags & NRF_MODEM_GNSS_PVT_FLAG_DEADLINE_MISSED) {
      LOG_INF("GNSS Blocked by LTE activity");
    } else if (pvt_data.flags & NRF_MODEM_GNSS_PVT_FLAG_NOT_ENOUGH_WINDOW_TIME) {
      LOG_INF("Insufficient GNSS time window");
    }
    break;

  case NRF_MODEM_GNSS_EVT_PERIODIC_WAKEUP:
    LOG_INF("GNSS has woken up");
    break;

  case NRF_MODEM_GNSS_EVT_SLEEP_AFTER_FIX:
    LOG_INF("GNSS enters sleep after fix");
    break;

  default:
    break;
  }
}

static void lte_handler(const struct lte_lc_evt *const evt)
{
  switch (evt->type) {
  case LTE_LC_EVT_NW_REG_STATUS:
    if (evt->nw_reg_status != LTE_LC_NW_REG_REGISTERED_HOME &&
        evt->nw_reg_status != LTE_LC_NW_REG_REGISTERED_ROAMING) {
      break;
    }
    LOG_INF("LTE registered: %s",
            evt->nw_reg_status == LTE_LC_NW_REG_REGISTERED_HOME ? "home network"
                                                                : "roaming");
    k_sem_give(&lte_connected);
    break;

  case LTE_LC_EVT_RRC_UPDATE:
    LOG_INF("RRC mode: %s",
            evt->rrc_mode == LTE_LC_RRC_MODE_CONNECTED ? "connected" : "idle");
    break;

  case LTE_LC_EVT_PSM_UPDATE:
    LOG_INF("PSM parameter update: TAU: %d s, Active time: %d s",
            evt->psm_cfg.tau, evt->psm_cfg.active_time);
    if (evt->psm_cfg.active_time == -1) {
      LOG_ERR("Network rejected PSM parameters. Failed to enable PSM");
    }
    break;

  case LTE_LC_EVT_EDRX_UPDATE:
    LOG_INF("eDRX parameter update: eDRX: %.2f s, PTW: %.2f s",
            (double)evt->edrx_cfg.edrx, (double)evt->edrx_cfg.ptw);
    break;

  default:
    break;
  }
}

static void date_time_event_handler(const struct date_time_evt *evt)
{
  if (evt->type == DATE_TIME_OBTAINED_MODEM ||
      evt->type == DATE_TIME_OBTAINED_NTP ||
      evt->type == DATE_TIME_OBTAINED_EXT) {
    LOG_INF("Time synced (source: %d)", evt->type);
    k_sem_give(&time_synced);
  }
}

static int modem_configure(void)
{
  int err;

  LOG_INF("Initializing modem library");
  err = nrf_modem_lib_init();
  if (err) {
    LOG_ERR("Failed to initialize the modem library, error: %d", err);
    return err;
  }

  date_time_register_handler(date_time_event_handler);

  /* Request eDRX and PSM from the network
   * This can also be done automatically using
   * Kconfig (CONFIG_LTE_PSM_REQ / CONFIG_LTE_EDRX_REQ)
   */
  err = lte_lc_psm_req(true);
  if (err) {
    LOG_ERR("lte_lc_psm_req, error: %d", err);
  }
  err = lte_lc_edrx_req(true);
  if (err) {
    LOG_ERR("lte_lc_edrx_req, error: %d", err);
  }

	LOG_INF("Connecting to LTE network");
	err = lte_lc_connect_async(lte_handler);
	if (err) {
		LOG_ERR("lte_lc_connect_async failed: %d", err);
		return err;
	}

	k_sem_take(&lte_connected, K_FOREVER);
	LOG_INF("Connected to LTE network");

  LOG_INF("Waiting for time sync...");
  err = k_sem_take(&time_synced, K_SECONDS(30));
  if (err) {
    LOG_ERR("Timed out waiting for time sync");
    return -ETIMEDOUT;
  }
  return 0;
}

static int gnss_init_and_start(void)
{
  int err;

  /* Activate gnss + lte */
  err = lte_lc_func_mode_set(LTE_LC_FUNC_MODE_NORMAL);
  if (err) {
    LOG_ERR("Failed to activate GNSS functional mode, error: %d", err);
    return err;
  }

  err = nrf_modem_gnss_event_handler_set(gnss_event_handler);
  if (err) {
    LOG_ERR("Failed to set GNSS event handler, error: %d", err);
    return err;
  }

  int32_t interval =
    IS_ENABLED(CONFIG_GNSS_SINGLE_FIX) ? 0 : CONFIG_GNSS_PERIODIC_INTERVAL;
  err = nrf_modem_gnss_fix_interval_set(interval);
  if (err) {
    LOG_ERR("Failed to set GNSS fix interval, error: %d", err);
    return err;
  }

  err = nrf_modem_gnss_fix_retry_set(CONFIG_GNSS_PERIODIC_TIMEOUT);
  if (err) {
    LOG_ERR("Failed to set GNSS fix retry, error: %d", err);
    return err;
  }

  LOG_INF("Starting GNSS");
  err = nrf_modem_gnss_start();
  if (err) {
    LOG_ERR("Failed to start GNSS, error: %d", err);
    return err;
  }

  gnss_start_time = k_uptime_get();

  return 0;
}

int main(void)
{
  int err;

  err = modem_configure();
  if (err) {
    LOG_ERR("Failed to configure the modem, error: %d", err);
    return err;
  }

  err = gnss_init_and_start();
  if (err) {
		LOG_ERR("Failed to initialize and start GNSS");
    return err;
  }

  while (1) {
    k_sleep(K_FOREVER);
  }

  return 0;
}
