#include <zephyr/kernel.h>
#include <modem/nrf_modem_lib.h>
#include <nrf_modem_gnss.h>
#include <zephyr/logging/log.h>
#include <modem/lte_lc.h>
#include <nrf_modem_at.h>
#include <stdint.h>

LOG_MODULE_REGISTER(nrf91_gnss);

static struct nrf_modem_gnss_pvt_data_frame pvt_data;

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

static int modem_configure(void)
{
  int err;

  LOG_INF("Initializing modem library");
  err = nrf_modem_lib_init();
  if (err) {
    LOG_ERR("Failed to initialize the modem library, error: %d", err);
    return err;
  }

  return 0;
}

static int gnss_init_and_start(void)
{
  int err;

  /* Activate only the gnss modem */
  err = lte_lc_func_mode_set(LTE_LC_FUNC_MODE_ACTIVATE_GNSS);
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
