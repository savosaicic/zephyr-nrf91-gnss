#include <zephyr/kernel.h>
#include <nrf_modem_gnss.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(agps_fake, LOG_LEVEL_INF);

/*
 * TOE/TOC: must be within ~2 hours of current GPS time of week.
 * raw = physical_seconds / 16
 * Update FAKE_TOC_TOE if rebuilding more than a few hours later.
 * GPS week 2409, ~223200 s into week (generated March 10 2026 ~15:00 UTC).
 */
#define FAKE_TOC_TOE   13950U      /* 223200 s / 16 */
#define FAKE_SQRT_A    2703380586U /* 5153.7 m^0.5 × 2^19 */
#define FAKE_I0        656019922   /* 55° × (2^31/180°) */
#define FAKE_OMEGA_DOT (-7281)
#define FAKE_DELTA_N   12596

/* UTC (flag 0x01) */
static struct nrf_modem_gnss_agnss_gps_data_utc fake_utc = {
  .a1         = 0,
  .a0         = 0,
  .tot        = 54,
  .wn_t       = 105, /* week 2409 mod 256 = 105 */
  .delta_tls  = 18,
  .wn_lsf     = 105,
  .dn         = 1,
  .delta_tlsf = 18,
};

/* KLOBUCHAR (flag 0x02) */
static struct nrf_modem_gnss_agnss_data_klobuchar fake_klobuchar = {
  .alpha0 = 12,
  .alpha1 = 4,
  .alpha2 = -1,
  .alpha3 = -2,
  .beta0  = 63,
  .beta1  = 10,
  .beta2  = -1,
  .beta3  = -9,
};

/* NEQUICK (flag 0x04)
 *
 * NeQuick is used for Galileo. The modem requests it regardless of whether
 * Galileo SVs are present. Injecting zeroed nominal values satisfies the
 * request without affecting GPS accuracy.
 *
 * struct nrf_modem_gnss_agnss_data_nequick:
 *   int16_t ai0   effective ionisation level 1st order  scale 2^-2   SFU
 *   int16_t ai1   effective ionisation level 2nd order  scale 2^-8   SFU/deg
 *   int16_t ai2   effective ionisation level 3rd order  scale 2^-15  SFU/deg²
 *   uint8_t storm_cond   ionospheric storm condition bitmask (0 = no storm)
 *   uint8_t storm_valid  storm condition valid flag (0 = not valid)
 *
 * Nominal mid-latitude values (quiet ionosphere):
 *   ai0 = 20.0 SFU  → raw = 20.0 / 2^-2  = 80
 *   ai1 = 0.0       → 0
 *   ai2 = 0.0       → 0
 */
static struct nrf_modem_gnss_agnss_data_nequick fake_nequick = {
  .ai0         = 80, /* 20.0 SFU / 2^-2 */
  .ai1         = 0,
  .ai2         = 0,
  .storm_cond  = 0, /* no ionospheric storm */
  .storm_valid = 0, /* storm data not valid */
};

/* GPS SYSTEM TIME + SV TOW (flag 0x08) */
static struct nrf_modem_gnss_agnss_gps_data_system_time_and_sv_tow
  fake_sys_time = {
    .date_day     = 16866, /* days since GPS epoch, approx March 10 2026 */
    .time_full_s  = 54000, /* full seconds of current day */
    .time_frac_ms = 0,     /* fractional milliseconds */
    .sv_mask      = 0,     /* no per-SV TOW data provided */
    /* sv_tow[] zero-initialized — modem ignores entries not set in sv_mask */
};

/* POSITION (flag 0x10) */
static struct nrf_modem_gnss_agnss_data_location fake_location = {
  .latitude  = 4553633, /* 48.8566° N: round(48.8566 × 2^23/90)  */
  .longitude = 109601,  /*  2.3522° E: round( 2.3522 × 2^24/360) */
  .altitude  = 35,      /* 35 m above WGS-84 ellipsoid            */

  /* Horizontal uncertainty: r = 10 × (1.1^K - 1) meters
   * K=26 → r ≈ 109 km  (circular, north-aligned) */
  .unc_semimajor = 26,
  .unc_semiminor = 26,
  .orientation_major =
    0, /* major axis pointing north (irrelevant for circle) */

  /* Altitude uncertainty: h = 45 × (1.025^K - 1) meters
   * 255 = "missing" — safest choice when we don't know altitude precisely */
  .unc_altitude = 255,

  /* Confidence: 68% = standard 1-sigma ellipse */
  .confidence = 68,
};

static struct nrf_modem_gnss_agnss_data_integrity fake_integrity = {
  .signal_count = 1,
  .signal =
    {
      {
        .signal_id      = NRF_MODEM_GNSS_SIGNAL_GPS_L1_CA,
        .integrity_mask = 0x0ULL, /* 0 = all GPS SVs healthy */
      },
    },
};

static struct nrf_modem_gnss_agnss_gps_data_ephemeris fake_ephem[4] = {
  {.sv_id     = 1,
   .health    = 0,
   .iodc      = 0x0001,
   .toc       = FAKE_TOC_TOE,
   .af0       = 0,
   .af1       = 0,
   .af2       = 0,
   .tgd       = 0,
   .ura       = 2,
   .toe       = FAKE_TOC_TOE,
   .w         = 0,
   .delta_n   = FAKE_DELTA_N,
   .m0        = 0,
   .omega_dot = FAKE_OMEGA_DOT,
   .e         = 0,
   .idot      = 0,
   .sqrt_a    = FAKE_SQRT_A,
   .i0        = FAKE_I0,
   .omega0    = 0,
   .crs       = 0,
   .cis       = 0,
   .cus       = 0,
   .crc       = 0,
   .cic       = 0,
   .cuc       = 0},
  {.sv_id     = 2,
   .health    = 0,
   .iodc      = 0x0002,
   .toc       = FAKE_TOC_TOE,
   .af0       = 0,
   .af1       = 0,
   .af2       = 0,
   .tgd       = 0,
   .ura       = 2,
   .toe       = FAKE_TOC_TOE,
   .w         = 0,
   .delta_n   = FAKE_DELTA_N,
   .m0        = 1073741824,
   .omega_dot = FAKE_OMEGA_DOT,
   .e         = 0,
   .idot      = 0,
   .sqrt_a    = FAKE_SQRT_A,
   .i0        = FAKE_I0,
   .omega0    = 715827882,
   .crs       = 0,
   .cis       = 0,
   .cus       = 0,
   .crc       = 0,
   .cic       = 0,
   .cuc       = 0},
  {.sv_id     = 3,
   .health    = 0,
   .iodc      = 0x0003,
   .toc       = FAKE_TOC_TOE,
   .af0       = 0,
   .af1       = 0,
   .af2       = 0,
   .tgd       = 0,
   .ura       = 2,
   .toe       = FAKE_TOC_TOE,
   .w         = 0,
   .delta_n   = FAKE_DELTA_N,
   .m0        = 2147483647,
   .omega_dot = FAKE_OMEGA_DOT,
   .e         = 0,
   .idot      = 0,
   .sqrt_a    = FAKE_SQRT_A,
   .i0        = FAKE_I0,
   .omega0    = 1431655765,
   .crs       = 0,
   .cis       = 0,
   .cus       = 0,
   .crc       = 0,
   .cic       = 0,
   .cuc       = 0},
  {.sv_id     = 4,
   .health    = 0,
   .iodc      = 0x0004,
   .toc       = FAKE_TOC_TOE,
   .af0       = 0,
   .af1       = 0,
   .af2       = 0,
   .tgd       = 0,
   .ura       = 2,
   .toe       = FAKE_TOC_TOE,
   .w         = 0,
   .delta_n   = FAKE_DELTA_N,
   .m0        = -1073741824,
   .omega_dot = FAKE_OMEGA_DOT,
   .e         = 0,
   .idot      = 0,
   .sqrt_a    = FAKE_SQRT_A,
   .i0        = FAKE_I0,
   .omega0    = 2147483647,
   .crs       = 0,
   .cis       = 0,
   .cus       = 0,
   .crc       = 0,
   .cic       = 0,
   .cuc       = 0},
};

static struct nrf_modem_gnss_agnss_gps_data_almanac fake_almanac[4] = {
  {.sv_id     = 1,
   .sv_health = 0,
   .wn        = 105,
   .toa       = 54,
   .ioda      = 0,
   .e         = 0,
   .delta_i   = 2919,
   .omega_dot = -92,
   .sqrt_a    = 10554726U,
   .omega0    = 0,
   .w         = 0,
   .m0        = 0,
   .af0       = 0,
   .af1       = 0},
  {.sv_id     = 2,
   .sv_health = 0,
   .wn        = 105,
   .toa       = 54,
   .ioda      = 0,
   .e         = 0,
   .delta_i   = 2919,
   .omega_dot = -92,
   .sqrt_a    = 10554726U,
   .omega0    = 1398101,
   .w         = 0,
   .m0        = 2097152,
   .af0       = 0,
   .af1       = 0},
  {.sv_id     = 3,
   .sv_health = 0,
   .wn        = 105,
   .toa       = 54,
   .ioda      = 0,
   .e         = 0,
   .delta_i   = 2919,
   .omega_dot = -92,
   .sqrt_a    = 10554726U,
   .omega0    = 2796202,
   .w         = 0,
   .m0        = 4194304,
   .af0       = 0,
   .af1       = 0},
  {.sv_id     = 4,
   .sv_health = 0,
   .wn        = 105,
   .toa       = 54,
   .ioda      = 0,
   .e         = 0,
   .delta_i   = 2919,
   .omega_dot = -92,
   .sqrt_a    = 10554726U,
   .omega0    = 4194304,
   .w         = 0,
   .m0        = -2097152,
   .af0       = 0,
   .af1       = 0},
};

#define INJECT(buf, type, label)                                               \
  do {                                                                         \
    int _r = nrf_modem_gnss_agnss_write(&(buf), sizeof(buf), (type));          \
    if (_r) {                                                                  \
      LOG_ERR(label " failed: %d", _r);                                        \
      return _r;                                                               \
    }                                                                          \
    LOG_INF(label " injected");                                                \
  } while (0)

int agps_fake_inject_all(const struct nrf_modem_gnss_agnss_data_frame *req)
{
  if (req->data_flags & NRF_MODEM_GNSS_AGNSS_GPS_UTC_REQUEST) {
    INJECT(fake_utc, NRF_MODEM_GNSS_AGNSS_GPS_UTC_PARAMETERS, "UTC");
  }
  if (req->data_flags & NRF_MODEM_GNSS_AGNSS_KLOBUCHAR_REQUEST) {
    INJECT(fake_klobuchar,
           NRF_MODEM_GNSS_AGNSS_KLOBUCHAR_IONOSPHERIC_CORRECTION, "Klobuchar");
  }
  if (req->data_flags & NRF_MODEM_GNSS_AGNSS_NEQUICK_REQUEST) {
    INJECT(fake_nequick, NRF_MODEM_GNSS_AGNSS_NEQUICK_IONOSPHERIC_CORRECTION,
           "NeQuick");
  }
  if (req->data_flags & NRF_MODEM_GNSS_AGNSS_GPS_SYS_TIME_AND_SV_TOW_REQUEST) {
    INJECT(fake_sys_time, NRF_MODEM_GNSS_AGNSS_GPS_SYSTEM_CLOCK_AND_TOWS,
           "Sys time");
  }
  if (req->data_flags & NRF_MODEM_GNSS_AGNSS_POSITION_REQUEST) {
    INJECT(fake_location, NRF_MODEM_GNSS_AGNSS_LOCATION, "Position");
  }
  if (req->data_flags & NRF_MODEM_GNSS_AGNSS_INTEGRITY_REQUEST) {
    INJECT(fake_integrity, NRF_MODEM_GNSS_AGNSS_INTEGRITY, "Integrity");
  }

  /* Ephemeris and almanac: iterate over systems requested */
  for (int s = 0; s < req->system_count; s++) {
    const struct nrf_modem_gnss_agnss_system_data_need *sys = &req->system[s];

    /* Only handle GPS */
    if (sys->system_id != NRF_MODEM_GNSS_SYSTEM_GPS) {
      LOG_INF("Skipping system_id %d (not GPS)", sys->system_id);
      continue;
    }

    LOG_INF("GPS ephe mask: 0x%08llX  alm mask: 0x%08llX", sys->sv_mask_ephe,
            sys->sv_mask_alm);

    /* Inject the 4 fakes ephemerides for any SV requested */
    if (sys->sv_mask_ephe) {
      for (int i = 0; i < ARRAY_SIZE(fake_ephem); i++) {
        int r =
          nrf_modem_gnss_agnss_write(&fake_ephem[i], sizeof(fake_ephem[i]),
                                     NRF_MODEM_GNSS_AGNSS_GPS_EPHEMERIDES);
        if (r) {
          LOG_ERR("Ephem SV%d failed: %d", fake_ephem[i].sv_id, r);
          return r;
        }
        LOG_INF("Ephem SV%d injected", fake_ephem[i].sv_id);
      }
    }

    /* Inject the 4 fakes almanacs for any SV requested */
    if (sys->sv_mask_alm) {
      for (int i = 0; i < ARRAY_SIZE(fake_almanac); i++) {
        int r =
          nrf_modem_gnss_agnss_write(&fake_almanac[i], sizeof(fake_almanac[i]),
                                     NRF_MODEM_GNSS_AGNSS_GPS_ALMANAC);
        if (r) {
          LOG_ERR("Alm SV%d failed: %d", fake_almanac[i].sv_id, r);
          return r;
        }
        LOG_INF("Alm SV%d injected", fake_almanac[i].sv_id);
      }
    }
  }

  LOG_INF("A-GNSS injection complete");
  return 0;
}
