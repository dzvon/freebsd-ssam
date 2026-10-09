/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Dezhi Wu
 *
 * Driver for the Microsoft Surface System Aggregator Module (SSAM).
 *
 * On newer Surface devices, the battery, the AC adapter and other
 * functions are not available through the ACPI EC.  An embedded
 * controller (the SAM) gives them through a serial link to an Intel
 * LPSS UART.  The ACPI device MSHW0084 ("SSH") tells which UART to use
 * and its line settings.
 *
 * This driver attaches to that LPSS UART on the PCI bus.  It operates
 * the UART directly (the device is a DesignWare 16550 with an LPSS
 * clock and reset block) and uses the Surface Serial Hub (SSH) protocol
 * to send requests to the SAM.
 *
 * SSH frame:
 *   SYN (0xaa 0x55)
 *   type (1), payload length (2, LE), sequence ID (1), CRC (2, LE)
 *   payload (length bytes), CRC (2, LE)
 *
 * The CRC is CRC-16/CCITT-FALSE.  The SAM acknowledges each DATA_SEQ
 * frame with an ACK frame that has the same sequence ID, and so must
 * the host.
 *
 * Reference: Documentation/driver-api/surface_aggregator/ssh.rst and
 * drivers/platform/surface/aggregator/ in the Linux kernel.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/endian.h>
#include <sys/kernel.h>
#include <sys/kthread.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/proc.h>
#include <sys/rman.h>
#include <sys/sx.h>
#include <sys/sysctl.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <contrib/dev/acpica/include/acpi.h>
#include <contrib/dev/acpica/include/accommon.h>

#include <dev/acpica/acpivar.h>
#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

#include "ssam.h"

/* UART registers (32-bit stride). */
#define	UART_RBR	0x00	/* receive buffer (read) */
#define	UART_THR	0x00	/* transmit holding (write) */
#define	UART_DLL	0x00	/* divisor latch low (DLAB=1) */
#define	UART_IER	0x04	/* interrupt enable */
#define	UART_DLM	0x04	/* divisor latch high (DLAB=1) */
#define	UART_IIR	0x08	/* interrupt identification (read) */
#define	UART_FCR	0x08	/* FIFO control (write) */
#define	UART_LCR	0x0c	/* line control */
#define	UART_MCR	0x10	/* modem control */
#define	UART_LSR	0x14	/* line status */
#define	UART_MSR	0x18	/* modem status */
#define	UART_USR	0x7c	/* DesignWare UART status */
#define	UART_CPR	0xf4	/* DesignWare component parameters */

#define	IER_ERBFI	0x01
#define	IER_ELSI	0x04
#define	IIR_NOPEND	0x01
#define	IIR_IMASK	0x0f
#define	IIR_MLSC	0x00
#define	IIR_RLS		0x06
#define	IIR_BUSY	0x07
#define	FCR_ENABLE	0x01
#define	FCR_RCV_RST	0x02
#define	FCR_XMT_RST	0x04
#define	FCR_RX_HALF	0x80
#define	LCR_8BITS	0x03
#define	LCR_DLAB	0x80
#define	MCR_DTR		0x01
#define	MCR_RTS		0x02
#define	MCR_OUT2	0x08
#define	MCR_AFE		0x20
#define	LSR_DR		0x01
#define	LSR_OE		0x02
#define	LSR_THRE	0x20

/* Intel LPSS private registers. */
#define	LPSS_CLK	0x200
#define	LPSS_CLK_EN	0x00000001
#define	LPSS_CLK_M_SHIFT 1
#define	LPSS_CLK_N_SHIFT 16
#define	LPSS_CLK_MN_MAX	0x7fff
#define	LPSS_CLK_UPDATE	0x80000000
#define	LPSS_RESETS	0x204
#define	LPSS_RESETS_ALL	0x7

/* Input clock of the LPSS UART on Ice Lake (as on Sunrise Point). */
#define	LPSS_CLK_RATE	120000000

/* SSH protocol. */
#define	SSH_SYN0		0xaa
#define	SSH_SYN1		0x55
#define	SSH_FRAME_HDR_LEN	8	/* SYN + type, len, seq + CRC */
#define	SSH_FRAME_DATA_NSQ	0x00
#define	SSH_FRAME_NAK		0x04
#define	SSH_FRAME_ACK		0x40
#define	SSH_FRAME_DATA_SEQ	0x80
#define	SSH_PLD_TYPE_CMD	0x80
#define	SSH_RQID_MIN		0x0100	/* lower IDs are for events */
#define	SSH_ACK_TIMEOUT		hz	/* per try */
#define	SSH_ACK_TRIES		3
#define	SSH_RSP_TIMEOUT		(3 * hz)

#define	SSAM_RING_SIZE		4096
#define	SSAM_RXBUF_SIZE		(SSH_FRAME_HDR_LEN + SSAM_MAX_PAYLOAD + 2)
#define	SSAM_TXBUF_SIZE		(SSH_FRAME_HDR_LEN + SSAM_CMD_HDR_LEN + 64 + 2)

enum ssam_req_state {
	SSAM_REQ_IDLE,
	SSAM_REQ_WAIT_ACK,
	SSAM_REQ_WAIT_RSP,
	SSAM_REQ_DONE,
};

struct ssam_softc {
	device_t		dev;
	ACPI_HANDLE		ssh_handle;
	struct resource		*mem;
	int			mem_rid;
	struct resource		*irq;
	int			irq_rid;
	void			*intr_cookie;
	int			msi;

	uint32_t		baud;
	u_int			tx_fifo;

	/* Protects the RX ring, the request state and the counters. */
	struct mtx		mtx;
	uint8_t			ring[SSAM_RING_SIZE];
	u_int			ring_head;
	u_int			ring_tail;

	/* RX parser state, used only by the RX thread. */
	uint8_t			rxbuf[SSAM_RXBUF_SIZE];
	size_t			rxlen;
	int			last_seq;	/* -1 if none */

	/* Serializes access to the transmitter. */
	struct sx		tx_lock;

	/* One request at a time. */
	struct sx		req_lock;
	enum ssam_req_state	req_state;
	uint8_t			req_seq;
	uint16_t		req_rqid;
	int			req_nak;
	uint8_t			req_rsp[SSAM_MAX_PAYLOAD];
	size_t			req_rsp_len;

	uint8_t			tx_seq;
	uint16_t		tx_rqid;

	struct proc		*rx_proc;
	int			dying;

	/* Statistics. */
	u_long			st_intr;
	u_long			st_rx_bytes;
	u_long			st_rx_frames;
	u_long			st_rx_crc;
	u_long			st_rx_junk;
	u_long			st_rx_overrun;
	u_long			st_rx_ringfull;
	u_long			st_rx_dup;
	u_long			st_rx_events;
	u_long			st_tx_frames;
	u_long			st_tx_retries;
	u_long			st_tx_timeouts;

	char			fw_version[16];
	int			debug;
};

static int	ssam_probe(device_t dev);
static int	ssam_attach(device_t dev);
static int	ssam_detach(device_t dev);

#define	RD4(sc, reg)		bus_read_4((sc)->mem, (reg))
#define	WR4(sc, reg, val)	bus_write_4((sc)->mem, (reg), (val))

static uint16_t
ssh_crc16(const uint8_t *buf, size_t len)
{
	uint16_t crc;
	int i;

	crc = 0xffff;
	while (len-- > 0) {
		crc ^= (uint16_t)*buf++ << 8;
		for (i = 0; i < 8; i++)
			crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
	}
	return (crc);
}

/*
 * ACPI: find the MSHW0084 device and its UART resource.
 */
struct ssam_acpi_ctx {
	ACPI_HANDLE	uart;		/* in: handle of our UART */
	ACPI_HANDLE	ssh;		/* out: handle of MSHW0084 */
	int		found;		/* out */
	uint32_t	baud;		/* out */
	int		flow;		/* out */
};

static ACPI_STATUS
ssam_acpi_res_cb(ACPI_RESOURCE *res, void *arg)
{
	struct ssam_acpi_ctx *ctx = arg;
	ACPI_RESOURCE_UART_SERIALBUS *uart;
	ACPI_HANDLE h;

	if (res->Type != ACPI_RESOURCE_TYPE_SERIAL_BUS)
		return (AE_OK);
	uart = &res->Data.UartSerialBus;
	if (uart->Type != ACPI_RESOURCE_SERIAL_TYPE_UART ||
	    uart->ResourceSource.StringPtr == NULL)
		return (AE_OK);
	if (ACPI_FAILURE(AcpiGetHandle(NULL, uart->ResourceSource.StringPtr,
	    &h)) || h != ctx->uart)
		return (AE_OK);

	ctx->found = 1;
	ctx->baud = uart->DefaultBaudRate;
	ctx->flow = uart->FlowControl;
	return (AE_CTRL_TERMINATE);
}

static ACPI_STATUS
ssam_acpi_dev_cb(ACPI_HANDLE h, UINT32 level, void *arg, void **ret)
{
	struct ssam_acpi_ctx *ctx = arg;

	AcpiWalkResources(h, "_CRS", ssam_acpi_res_cb, ctx);
	if (!ctx->found)
		return (AE_OK);
	ctx->ssh = h;
	return (AE_CTRL_TERMINATE);
}

static int
ssam_acpi_lookup(device_t dev, struct ssam_acpi_ctx *ctx)
{
	memset(ctx, 0, sizeof(*ctx));
	ctx->uart = acpi_get_handle(dev);
	if (ctx->uart == NULL)
		return (ENXIO);
	AcpiGetDevices("MSHW0084", ssam_acpi_dev_cb, ctx, NULL);
	return (ctx->found ? 0 : ENXIO);
}

/*
 * UART access.
 */
static int
ssam_uart_set_lcr(struct ssam_softc *sc, uint32_t lcr)
{
	int i;

	/*
	 * The DesignWare UART ignores a write to LCR while it is busy.
	 * Clear the FIFOs and try again.
	 */
	for (i = 0; i < 1000; i++) {
		WR4(sc, UART_LCR, lcr);
		if ((RD4(sc, UART_LCR) & 0xff) == lcr)
			return (0);
		WR4(sc, UART_FCR, FCR_ENABLE | FCR_RCV_RST | FCR_XMT_RST);
		(void)RD4(sc, UART_RBR);
		(void)RD4(sc, UART_USR);
		DELAY(10);
	}
	return (ETIMEDOUT);
}

static int
ssam_uart_init(struct ssam_softc *sc)
{
	uint64_t clk, g, a, b, t;
	uint32_t m, n, div, cpr;
	int error;

	/* Take the LPSS block out of reset. */
	WR4(sc, LPSS_RESETS, 0);
	DELAY(10);
	WR4(sc, LPSS_RESETS, LPSS_RESETS_ALL);
	DELAY(10);

	/*
	 * Set the fractional divider so that the UART clock is
	 * 16 * baud.  Then the divisor latch is 1.
	 */
	clk = (uint64_t)sc->baud * 16;
	a = clk;
	b = LPSS_CLK_RATE;
	while (b != 0) {
		t = a % b;
		a = b;
		b = t;
	}
	g = a;
	m = clk / g;
	n = LPSS_CLK_RATE / g;
	if (m > LPSS_CLK_MN_MAX || n > LPSS_CLK_MN_MAX || m > n) {
		device_printf(sc->dev, "no clock divider for %u baud\n",
		    sc->baud);
		return (ENXIO);
	}
	WR4(sc, LPSS_CLK, (m << LPSS_CLK_M_SHIFT) | (n << LPSS_CLK_N_SHIFT));
	WR4(sc, LPSS_CLK, LPSS_CLK_EN | (m << LPSS_CLK_M_SHIFT) |
	    (n << LPSS_CLK_N_SHIFT) | LPSS_CLK_UPDATE);
	DELAY(10);
	div = 1;

	/* Stop the SAM from sending while we set up the UART. */
	WR4(sc, UART_MCR, 0);
	WR4(sc, UART_IER, 0);
	WR4(sc, UART_FCR, FCR_ENABLE | FCR_RCV_RST | FCR_XMT_RST);

	if ((error = ssam_uart_set_lcr(sc, LCR_DLAB | LCR_8BITS)) != 0)
		goto busy;
	WR4(sc, UART_DLL, div & 0xff);
	WR4(sc, UART_DLM, (div >> 8) & 0xff);
	if ((error = ssam_uart_set_lcr(sc, LCR_8BITS)) != 0)
		goto busy;

	cpr = RD4(sc, UART_CPR);
	sc->tx_fifo = ((cpr >> 16) & 0xff) * 16;
	if (sc->tx_fifo == 0)
		sc->tx_fifo = 16;
	if (sc->tx_fifo > 64)
		sc->tx_fifo = 64;

	WR4(sc, UART_FCR, FCR_ENABLE | FCR_RCV_RST | FCR_XMT_RST |
	    FCR_RX_HALF);
	(void)RD4(sc, UART_LSR);
	(void)RD4(sc, UART_RBR);
	(void)RD4(sc, UART_IIR);
	(void)RD4(sc, UART_MSR);
	(void)RD4(sc, UART_USR);

	/* Hardware flow control: the UART controls RTS from the FIFO. */
	WR4(sc, UART_MCR, MCR_DTR | MCR_RTS | MCR_OUT2 | MCR_AFE);
	return (0);

busy:
	device_printf(sc->dev, "UART stays busy, cannot set LCR\n");
	return (error);
}

/* Move received bytes from the UART FIFO to the ring.  Lock held. */
static int
ssam_uart_drain(struct ssam_softc *sc)
{
	uint32_t lsr;
	u_int next;
	int n;

	mtx_assert(&sc->mtx, MA_OWNED);
	n = 0;
	while ((lsr = RD4(sc, UART_LSR)) & LSR_DR) {
		if (lsr & LSR_OE)
			sc->st_rx_overrun++;
		next = (sc->ring_head + 1) % SSAM_RING_SIZE;
		if (next == sc->ring_tail) {
			(void)RD4(sc, UART_RBR);
			sc->st_rx_ringfull++;
			continue;
		}
		sc->ring[sc->ring_head] = RD4(sc, UART_RBR) & 0xff;
		sc->ring_head = next;
		n++;
	}
	sc->st_rx_bytes += n;
	return (n);
}

static void
ssam_intr(void *arg)
{
	struct ssam_softc *sc = arg;
	uint32_t iir;
	int i;

	mtx_lock(&sc->mtx);
	sc->st_intr++;
	for (i = 0; i < 64; i++) {
		iir = RD4(sc, UART_IIR);
		if (iir & IIR_NOPEND)
			break;
		switch (iir & IIR_IMASK) {
		case IIR_BUSY:
			(void)RD4(sc, UART_USR);
			break;
		case IIR_RLS:
			(void)RD4(sc, UART_LSR);
			break;
		case IIR_MLSC:
			(void)RD4(sc, UART_MSR);
			break;
		default:
			break;
		}
		ssam_uart_drain(sc);
	}
	if (sc->ring_head != sc->ring_tail)
		wakeup(&sc->ring);
	mtx_unlock(&sc->mtx);
}

/* Send bytes.  The tx_lock must be held. */
static int
ssam_uart_tx(struct ssam_softc *sc, const uint8_t *buf, size_t len)
{
	size_t i, n;
	int timo;

	sx_assert(&sc->tx_lock, SA_XLOCKED);
	while (len > 0) {
		/* CTS can hold the transmitter, so wait up to 100 ms. */
		for (timo = 100000; timo > 0; timo--) {
			if (RD4(sc, UART_LSR) & LSR_THRE)
				break;
			DELAY(1);
		}
		if (timo == 0) {
			sc->st_tx_timeouts++;
			return (ETIMEDOUT);
		}
		n = MIN(len, sc->tx_fifo);
		for (i = 0; i < n; i++)
			WR4(sc, UART_THR, buf[i]);
		buf += n;
		len -= n;
	}
	return (0);
}

/*
 * SSH frames.
 */
static size_t
ssh_put_frame_hdr(uint8_t *p, uint8_t type, uint16_t len, uint8_t seq)
{
	p[0] = SSH_SYN0;
	p[1] = SSH_SYN1;
	p[2] = type;
	le16enc(&p[3], len);
	p[5] = seq;
	le16enc(&p[6], ssh_crc16(&p[2], 4));
	return (SSH_FRAME_HDR_LEN);
}

static void
ssam_send_ack(struct ssam_softc *sc, uint8_t type, uint8_t seq)
{
	uint8_t buf[SSH_FRAME_HDR_LEN + 2];

	ssh_put_frame_hdr(buf, type, 0, seq);
	le16enc(&buf[SSH_FRAME_HDR_LEN], ssh_crc16(NULL, 0));
	sx_xlock(&sc->tx_lock);
	(void)ssam_uart_tx(sc, buf, sizeof(buf));
	sx_xunlock(&sc->tx_lock);
}

static void
ssam_rx_payload(struct ssam_softc *sc, const uint8_t *pld, size_t len)
{
	uint16_t rqid;

	if (len < SSAM_CMD_HDR_LEN || pld[0] != SSH_PLD_TYPE_CMD)
		return;
	rqid = le16dec(&pld[5]);

	mtx_lock(&sc->mtx);
	if ((sc->req_state == SSAM_REQ_WAIT_ACK ||
	    sc->req_state == SSAM_REQ_WAIT_RSP) && rqid == sc->req_rqid) {
		/* A response also tells us that our frame arrived. */
		len -= SSAM_CMD_HDR_LEN;
		memcpy(sc->req_rsp, pld + SSAM_CMD_HDR_LEN,
		    MIN(len, sizeof(sc->req_rsp)));
		sc->req_rsp_len = len;
		sc->req_state = SSAM_REQ_DONE;
		wakeup(&sc->req_state);
		mtx_unlock(&sc->mtx);
		return;
	}
	sc->st_rx_events++;
	mtx_unlock(&sc->mtx);

	if (sc->debug)
		device_printf(sc->dev, "event: tc %#x tid %#x sid %#x "
		    "iid %#x rqid %#x cid %#x len %zu\n", pld[1], pld[2],
		    pld[3], pld[4], rqid, pld[7], len - SSAM_CMD_HDR_LEN);
}

static void
ssam_rx_frame(struct ssam_softc *sc, uint8_t type, uint8_t seq,
    const uint8_t *pld, size_t len)
{
	if (sc->debug > 1)
		device_printf(sc->dev, "rx frame: type %#x seq %u len %zu\n",
		    type, seq, len);
	sc->st_rx_frames++;

	switch (type) {
	case SSH_FRAME_ACK:
	case SSH_FRAME_NAK:
		mtx_lock(&sc->mtx);
		if (sc->req_state == SSAM_REQ_WAIT_ACK && seq == sc->req_seq) {
			if (type == SSH_FRAME_ACK)
				sc->req_state = SSAM_REQ_WAIT_RSP;
			else
				sc->req_nak = 1;
			wakeup(&sc->req_state);
		}
		mtx_unlock(&sc->mtx);
		break;
	case SSH_FRAME_DATA_SEQ:
		ssam_send_ack(sc, SSH_FRAME_ACK, seq);
		if (seq == sc->last_seq) {
			/* The SAM did not get our ACK and sent it again. */
			sc->st_rx_dup++;
			break;
		}
		sc->last_seq = seq;
		ssam_rx_payload(sc, pld, len);
		break;
	case SSH_FRAME_DATA_NSQ:
		ssam_rx_payload(sc, pld, len);
		break;
	default:
		break;
	}
}

static void
ssam_rx_consume(struct ssam_softc *sc, size_t n)
{
	memmove(sc->rxbuf, sc->rxbuf + n, sc->rxlen - n);
	sc->rxlen -= n;
}

static void
ssam_rx_parse(struct ssam_softc *sc)
{
	const uint8_t *b;
	size_t off, len, total;

	for (;;) {
		b = sc->rxbuf;
		off = 0;
		while (off + 1 < sc->rxlen &&
		    (b[off] != SSH_SYN0 || b[off + 1] != SSH_SYN1))
			off++;
		if (off + 1 == sc->rxlen && b[off] != SSH_SYN0)
			off++;
		if (off > 0) {
			sc->st_rx_junk += off;
			ssam_rx_consume(sc, off);
		}
		if (sc->rxlen < SSH_FRAME_HDR_LEN)
			return;
		if (ssh_crc16(&b[2], 4) != le16dec(&b[6])) {
			sc->st_rx_crc++;
			ssam_rx_consume(sc, 1);
			continue;
		}
		len = le16dec(&b[3]);
		if (len > SSAM_MAX_PAYLOAD) {
			ssam_rx_consume(sc, 1);
			continue;
		}
		total = SSH_FRAME_HDR_LEN + len + 2;
		if (sc->rxlen < total)
			return;
		if (ssh_crc16(&b[SSH_FRAME_HDR_LEN], len) !=
		    le16dec(&b[SSH_FRAME_HDR_LEN + len])) {
			sc->st_rx_crc++;
			ssam_rx_consume(sc, 1);
			continue;
		}
		ssam_rx_frame(sc, b[2], b[5], &b[SSH_FRAME_HDR_LEN], len);
		ssam_rx_consume(sc, total);
	}
}

static void
ssam_rx_thread(void *arg)
{
	struct ssam_softc *sc = arg;
	size_t n;

	mtx_lock(&sc->mtx);
	while (!sc->dying) {
		/* Poll too, in case the interrupt does not come. */
		ssam_uart_drain(sc);
		n = 0;
		while (sc->ring_tail != sc->ring_head &&
		    sc->rxlen < sizeof(sc->rxbuf)) {
			sc->rxbuf[sc->rxlen++] = sc->ring[sc->ring_tail];
			sc->ring_tail = (sc->ring_tail + 1) % SSAM_RING_SIZE;
			n++;
		}
		if (n == 0) {
			msleep(&sc->ring, &sc->mtx, 0, "ssamrx",
			    MAX(1, hz / 50));
			continue;
		}
		mtx_unlock(&sc->mtx);
		ssam_rx_parse(sc);
		mtx_lock(&sc->mtx);
	}
	sc->rx_proc = NULL;
	wakeup(&sc->rx_proc);
	mtx_unlock(&sc->mtx);
	kproc_exit(0);
}

/*
 * Requests.
 */
int
ssam_request(device_t dev, const struct ssam_rqst *rq, void *rsp,
    size_t rsp_max, size_t *rsp_len)
{
	struct ssam_softc *sc = device_get_softc(dev);
	uint8_t buf[SSAM_TXBUF_SIZE], *cmd;
	size_t flen, plen;
	int error, try, timo;

	plen = SSAM_CMD_HDR_LEN + rq->plen;
	if (rq->plen > SSAM_TXBUF_SIZE - SSH_FRAME_HDR_LEN -
	    SSAM_CMD_HDR_LEN - 2)
		return (EINVAL);

	sx_xlock(&sc->req_lock);
	if (sc->dying) {
		sx_xunlock(&sc->req_lock);
		return (ENXIO);
	}

	mtx_lock(&sc->mtx);
	sc->req_seq = sc->tx_seq++;
	if (sc->tx_rqid < SSH_RQID_MIN)
		sc->tx_rqid = SSH_RQID_MIN;
	sc->req_rqid = sc->tx_rqid++;
	sc->req_nak = 0;
	sc->req_rsp_len = 0;
	sc->req_state = SSAM_REQ_WAIT_ACK;
	mtx_unlock(&sc->mtx);

	flen = ssh_put_frame_hdr(buf, SSH_FRAME_DATA_SEQ, plen, sc->req_seq);
	cmd = &buf[flen];
	cmd[0] = SSH_PLD_TYPE_CMD;
	cmd[1] = rq->tc;
	cmd[2] = rq->tid;
	cmd[3] = SSAM_TID_HOST;
	cmd[4] = rq->iid;
	le16enc(&cmd[5], sc->req_rqid);
	cmd[7] = rq->cid;
	if (rq->plen > 0)
		memcpy(&cmd[SSAM_CMD_HDR_LEN], rq->pld, rq->plen);
	le16enc(&cmd[plen], ssh_crc16(cmd, plen));
	flen += plen + 2;

	error = ETIMEDOUT;
	for (try = 0; try < SSH_ACK_TRIES; try++) {
		if (try > 0)
			sc->st_tx_retries++;
		sx_xlock(&sc->tx_lock);
		error = ssam_uart_tx(sc, buf, flen);
		sx_xunlock(&sc->tx_lock);
		if (error != 0)
			continue;
		sc->st_tx_frames++;

		mtx_lock(&sc->mtx);
		sc->req_nak = 0;
		timo = SSH_ACK_TIMEOUT;
		while (sc->req_state == SSAM_REQ_WAIT_ACK && !sc->req_nak) {
			if (msleep(&sc->req_state, &sc->mtx, 0, "ssamack",
			    timo) == EWOULDBLOCK)
				break;
		}
		if (sc->req_state != SSAM_REQ_WAIT_ACK) {
			mtx_unlock(&sc->mtx);
			error = 0;
			break;
		}
		mtx_unlock(&sc->mtx);
		error = ETIMEDOUT;
	}

	mtx_lock(&sc->mtx);
	if (error == 0) {
		while (sc->req_state == SSAM_REQ_WAIT_RSP) {
			if (msleep(&sc->req_state, &sc->mtx, 0, "ssamrsp",
			    SSH_RSP_TIMEOUT) == EWOULDBLOCK)
				break;
		}
		if (sc->req_state != SSAM_REQ_DONE)
			error = ETIMEDOUT;
	}
	if (error == 0) {
		if (rsp != NULL)
			memcpy(rsp, sc->req_rsp,
			    MIN(rsp_max, MIN(sc->req_rsp_len,
			    sizeof(sc->req_rsp))));
		if (rsp_len != NULL)
			*rsp_len = sc->req_rsp_len;
	}
	sc->req_state = SSAM_REQ_IDLE;
	mtx_unlock(&sc->mtx);

	sx_xunlock(&sc->req_lock);
	if (error != 0 && sc->debug)
		device_printf(dev, "request tc %#x cid %#x failed: %d\n",
		    rq->tc, rq->cid, error);
	return (error);
}

ACPI_HANDLE
ssam_get_acpi_handle(device_t dev)
{
	struct ssam_softc *sc = device_get_softc(dev);

	return (sc->ssh_handle);
}

static int
ssam_get_fw_version(struct ssam_softc *sc)
{
	struct ssam_rqst rq = {
		.tc = SSAM_TC_SAM,
		.tid = SSAM_TID_SAM,
		.iid = 0x00,
		.cid = 0x13,
	};
	uint8_t rsp[4];
	size_t len;
	uint32_t v;
	int error;

	error = ssam_request(sc->dev, &rq, rsp, sizeof(rsp), &len);
	if (error != 0)
		return (error);
	if (len < sizeof(rsp))
		return (EIO);
	v = le32dec(rsp);
	snprintf(sc->fw_version, sizeof(sc->fw_version), "%u.%u.%u",
	    (v >> 24) & 0xff, (v >> 12) & 0xfff, v & 0xfff);
	return (0);
}

/*
 * Device interface.
 */
static int
ssam_probe(device_t dev)
{
	struct ssam_acpi_ctx ctx;

	if (pci_get_vendor(dev) != 0x8086 || pci_get_device(dev) != 0x34a8)
		return (ENXIO);
	/* Only take the UART that the SAM uses. */
	if (ssam_acpi_lookup(dev, &ctx) != 0)
		return (ENXIO);
	device_set_desc(dev, "Surface System Aggregator Module");
	return (BUS_PROBE_SPECIFIC);
}

static void
ssam_add_sysctls(struct ssam_softc *sc)
{
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid_list *list;

	ctx = device_get_sysctl_ctx(sc->dev);
	list = SYSCTL_CHILDREN(device_get_sysctl_tree(sc->dev));

	SYSCTL_ADD_STRING(ctx, list, OID_AUTO, "fw_version", CTLFLAG_RD,
	    sc->fw_version, 0, "SAM firmware version");
	SYSCTL_ADD_U32(ctx, list, OID_AUTO, "baud", CTLFLAG_RD,
	    &sc->baud, 0, "UART baud rate");
	SYSCTL_ADD_INT(ctx, list, OID_AUTO, "msi", CTLFLAG_RD,
	    &sc->msi, 0, "MSI in use");
	SYSCTL_ADD_INT(ctx, list, OID_AUTO, "debug", CTLFLAG_RW,
	    &sc->debug, 0, "Debug level (0-2)");
#define	STAT(name, field, desc)						\
	SYSCTL_ADD_ULONG(ctx, list, OID_AUTO, name, CTLFLAG_RD,		\
	    &sc->field, desc)
	STAT("intr", st_intr, "Interrupts");
	STAT("rx_bytes", st_rx_bytes, "Bytes received");
	STAT("rx_frames", st_rx_frames, "Frames received");
	STAT("rx_crc_errors", st_rx_crc, "Frames with a bad CRC");
	STAT("rx_junk", st_rx_junk, "Bytes outside of frames");
	STAT("rx_overruns", st_rx_overrun, "UART overruns");
	STAT("rx_ring_full", st_rx_ringfull, "Bytes lost, ring full");
	STAT("rx_duplicates", st_rx_dup, "Frames received again");
	STAT("rx_events", st_rx_events, "Events and unknown responses");
	STAT("tx_frames", st_tx_frames, "Frames sent");
	STAT("tx_retries", st_tx_retries, "Frames sent again");
	STAT("tx_timeouts", st_tx_timeouts, "Transmitter timeouts");
#undef STAT
}

static int
ssam_attach(device_t dev)
{
	struct ssam_softc *sc = device_get_softc(dev);
	struct ssam_acpi_ctx ctx;
	int count, error;

	sc->dev = dev;
	sc->last_seq = -1;
	sc->tx_rqid = SSH_RQID_MIN;
	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);
	sx_init(&sc->tx_lock, "ssam tx");
	sx_init(&sc->req_lock, "ssam req");

	if (ssam_acpi_lookup(dev, &ctx) != 0) {
		device_printf(dev, "cannot find the MSHW0084 device\n");
		error = ENXIO;
		goto fail;
	}
	sc->ssh_handle = ctx.ssh;
	sc->baud = ctx.baud;
	TUNABLE_INT_FETCH("hw.ssam.baud", &sc->baud);
	if (sc->baud == 0) {
		device_printf(dev, "no baud rate in _CRS\n");
		error = ENXIO;
		goto fail;
	}

	sc->mem_rid = PCIR_BAR(0);
	sc->mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &sc->mem_rid,
	    RF_ACTIVE);
	if (sc->mem == NULL) {
		device_printf(dev, "cannot allocate memory\n");
		error = ENXIO;
		goto fail;
	}

	if ((error = ssam_uart_init(sc)) != 0)
		goto fail;

	count = 1;
	if (pci_alloc_msi(dev, &count) == 0) {
		sc->msi = 1;
		sc->irq_rid = 1;
	}
	sc->irq = bus_alloc_resource_any(dev, SYS_RES_IRQ, &sc->irq_rid,
	    RF_ACTIVE | (sc->msi ? 0 : RF_SHAREABLE));
	if (sc->irq == NULL) {
		device_printf(dev, "cannot allocate interrupt\n");
		error = ENXIO;
		goto fail;
	}
	error = bus_setup_intr(dev, sc->irq, INTR_TYPE_MISC | INTR_MPSAFE,
	    NULL, ssam_intr, sc, &sc->intr_cookie);
	if (error != 0) {
		device_printf(dev, "cannot set up interrupt\n");
		goto fail;
	}

	error = kproc_create(ssam_rx_thread, sc, &sc->rx_proc, 0, 0,
	    "%s rx", device_get_nameunit(dev));
	if (error != 0) {
		device_printf(dev, "cannot start the RX thread\n");
		goto fail;
	}

	WR4(sc, UART_IER, IER_ERBFI | IER_ELSI);
	ssam_add_sysctls(sc);

	device_printf(dev, "%u baud, flow control %#x, FIFO %u, %s\n",
	    sc->baud, ctx.flow, sc->tx_fifo, sc->msi ? "MSI" : "INTx");

	error = ssam_get_fw_version(sc);
	if (error != 0) {
		device_printf(dev, "SAM does not answer (error %d)\n", error);
		/* Stay attached, so that the counters can show why. */
		return (0);
	}
	device_printf(dev, "SAM firmware version %s\n", sc->fw_version);

	device_add_child(dev, "battery", DEVICE_UNIT_ANY);
	bus_attach_children(dev);
	return (0);

fail:
	ssam_detach(dev);
	return (error);
}

static int
ssam_detach(device_t dev)
{
	struct ssam_softc *sc = device_get_softc(dev);
	int error;

	error = bus_generic_detach(dev);
	if (error != 0)
		return (error);

	sx_xlock(&sc->req_lock);
	sc->dying = 1;
	sx_xunlock(&sc->req_lock);

	mtx_lock(&sc->mtx);
	while (sc->rx_proc != NULL) {
		wakeup(&sc->ring);
		msleep(&sc->rx_proc, &sc->mtx, 0, "ssamdet", hz);
	}
	mtx_unlock(&sc->mtx);

	if (sc->mem != NULL) {
		WR4(sc, UART_IER, 0);
		WR4(sc, UART_MCR, 0);
	}
	if (sc->intr_cookie != NULL)
		bus_teardown_intr(dev, sc->irq, sc->intr_cookie);
	if (sc->irq != NULL)
		bus_release_resource(dev, SYS_RES_IRQ, sc->irq_rid, sc->irq);
	if (sc->msi)
		pci_release_msi(dev);
	if (sc->mem != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, sc->mem_rid,
		    sc->mem);

	sx_destroy(&sc->req_lock);
	sx_destroy(&sc->tx_lock);
	mtx_destroy(&sc->mtx);
	return (0);
}

static device_method_t ssam_methods[] = {
	DEVMETHOD(device_probe,		ssam_probe),
	DEVMETHOD(device_attach,	ssam_attach),
	DEVMETHOD(device_detach,	ssam_detach),

	DEVMETHOD(bus_print_child,	bus_generic_print_child),

	DEVMETHOD_END
};

static driver_t ssam_driver = {
	"ssam",
	ssam_methods,
	sizeof(struct ssam_softc),
};

DRIVER_MODULE(ssam, pci, ssam_driver, 0, 0);
MODULE_DEPEND(ssam, pci, 1, 1, 1);
MODULE_DEPEND(ssam, acpi, 1, 1, 1);
MODULE_VERSION(ssam, 1);
