/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Dezhi Wu
 *
 * Driver for the Microsoft Surface System Aggregator Module (SSAM).
 */

#ifndef _SSAM_H_
#define _SSAM_H_

/* Target categories. */
#define	SSAM_TC_SAM		0x01
#define	SSAM_TC_BAT		0x02

/* Target IDs. */
#define	SSAM_TID_HOST		0x00
#define	SSAM_TID_SAM		0x01

/* Maximum length of a frame payload that we accept. */
#define	SSAM_MAX_PAYLOAD	512

/* Size of the command header in a payload. */
#define	SSAM_CMD_HDR_LEN	8

struct ssam_rqst {
	uint8_t		tc;	/* target category */
	uint8_t		tid;	/* target ID */
	uint8_t		iid;	/* instance ID */
	uint8_t		cid;	/* command ID */
	const void	*pld;	/* request payload */
	size_t		plen;	/* request payload length */
};

/*
 * Send a request to the SAM and wait for the response.  The response
 * data is copied into rsp (at most rsp_max bytes).  The full length of
 * the response data is put into *rsp_len.  The caller can sleep.
 */
int	ssam_request(device_t dev, const struct ssam_rqst *rq, void *rsp,
	    size_t rsp_max, size_t *rsp_len);

/* Return the handle of the ACPI device MSHW0084 ("SSH"). */
ACPI_HANDLE ssam_get_acpi_handle(device_t dev);

#endif /* _SSAM_H_ */
