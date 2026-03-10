#ifndef FAKE_AGPS_DATA
#define FAKE_AGPS_DATA

#include <nrf_modem_gnss.h>

int agps_fake_inject_all(const struct nrf_modem_gnss_agnss_data_frame *req);

#endif /* !FAKE_AGPS_DATA */
