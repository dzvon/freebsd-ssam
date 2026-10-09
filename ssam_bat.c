/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 Dezhi Wu
 *
 * Battery and AC adapter of the Surface System Aggregator Module.
 *
 * The SAM gives the ACPI battery methods (_STA, _BIX, _BST) as requests
 * in the BAT target category.  This driver registers with the ACPI
 * battery framework, so that acpiconf(8), apm(8) and the
 * hw.acpi.battery sysctls work.  If there is no ACPI AC adapter, it
 * also adds hw.acpi.acline.
 *
 * The SAM can send events when the battery or the AC adapter changes,
 * but this driver does not enable them yet.  A task reads the values
 * at a fixed interval and sends the notifications that acpi_acad(4)
 * and acpi_cmbat(4) send: "ACAD" with the AC status, and "CMBAT" with
 * 0x80 when the battery status changes.  powerd(8) and devd(8) use the
 * "ACAD" notifications.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/endian.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/power.h>
#include <sys/sx.h>
#include <sys/sysctl.h>
#include <sys/taskqueue.h>
#include <sys/time.h>

#include <contrib/dev/acpica/include/acpi.h>
#include <contrib/dev/acpica/include/accommon.h>

#include <dev/acpica/acpivar.h>
#include <dev/acpica/acpiio.h>

#include "ssam.h"

#define	BAT_CID_STA	0x01
#define	BAT_CID_BIX	0x02
#define	BAT_CID_BST	0x03
#define	BAT_CID_PSRC	0x0d

#define	BAT_STA_PRESENT	0x10

/* Length of the _BIX payload from the SAM (packed). */
#define	BAT_BIX_LEN	119
#define	BAT_BST_LEN	16

/* Time to keep the _BST data, in seconds. */
#define	BAT_BST_EXPIRE	1

/* Default interval of the poll task, in seconds. */
#define	BAT_POLL_INTERVAL 5

/* Notify value of acpi_cmbat(4) for a battery status change. */
#define	BAT_NOTIFY_STATUS 0x80

struct ssam_bat_softc {
	device_t		dev;
	struct sx		lock;
	uint8_t			tid;
	uint8_t			iid;
	struct acpi_bix		bix;
	time_t			bix_time;
	int			bix_valid;
	struct acpi_bst		bst;
	time_t			bst_time;
	int			bst_valid;
	struct sysctl_ctx_list	acline_ctx;
	int			acline_added;

	/* Poll task. */
	ACPI_HANDLE		handle;
	struct taskqueue	*tq;
	struct timeout_task	poll_task;
	int			poll_interval;
	volatile int		detaching;
	int			acline;		/* -1 if unknown */
	uint32_t		state;		/* last _BST state */
};

static int
ssam_bat_rq(struct ssam_bat_softc *sc, uint8_t cid, void *rsp, size_t len)
{
	struct ssam_rqst rq = {
		.tc = SSAM_TC_BAT,
		.tid = sc->tid,
		.iid = sc->iid,
		.cid = cid,
	};
	size_t rlen;
	int error;

	error = ssam_request(device_get_parent(sc->dev), &rq, rsp, len, &rlen);
	if (error != 0)
		return (error);
	if (rlen < len)
		return (EIO);
	return (0);
}

static int
ssam_bat_present(struct ssam_bat_softc *sc)
{
	uint8_t rsp[4];

	if (ssam_bat_rq(sc, BAT_CID_STA, rsp, sizeof(rsp)) != 0)
		return (0);
	return ((le32dec(rsp) & BAT_STA_PRESENT) != 0);
}

static void
ssam_bat_strcpy(char *dst, const uint8_t *src, size_t len)
{
	len = MIN(len, ACPI_CMBAT_MAXSTRLEN - 1);
	memcpy(dst, src, len);
	dst[len] = '\0';
}

static int
ssam_bat_update_bix(struct ssam_bat_softc *sc)
{
	uint8_t p[BAT_BIX_LEN];
	struct acpi_bix *bix = &sc->bix;
	int error;

	sx_assert(&sc->lock, SA_XLOCKED);
	error = ssam_bat_rq(sc, BAT_CID_BIX, p, sizeof(p));
	if (error != 0)
		return (error);

	memset(bix, 0, sizeof(*bix));
	bix->rev = p[0] == 0 ? ACPI_BIX_REV_0 : ACPI_BIX_REV_1;
	bix->units = le32dec(&p[1]);
	bix->dcap = le32dec(&p[5]);
	bix->lfcap = le32dec(&p[9]);
	bix->btech = le32dec(&p[13]);
	bix->dvol = le32dec(&p[17]);
	bix->wcap = le32dec(&p[21]);
	bix->lcap = le32dec(&p[25]);
	bix->cycles = le32dec(&p[29]);
	bix->accuracy = le32dec(&p[33]);
	bix->stmax = le32dec(&p[37]);
	bix->stmin = le32dec(&p[41]);
	bix->aimax = le32dec(&p[45]);
	bix->aimin = le32dec(&p[49]);
	bix->gra1 = le32dec(&p[53]);
	bix->gra2 = le32dec(&p[57]);
	ssam_bat_strcpy(bix->model, &p[61], 21);
	ssam_bat_strcpy(bix->serial, &p[82], 11);
	ssam_bat_strcpy(bix->type, &p[93], 5);
	ssam_bat_strcpy(bix->oeminfo, &p[98], 21);
	bix->scap = ACPI_BIX_SCAP_NO;

	sc->bix_time = time_uptime;
	sc->bix_valid = 1;
	return (0);
}

static int
ssam_bat_update_bst(struct ssam_bat_softc *sc)
{
	uint8_t p[BAT_BST_LEN];
	int error;

	sx_assert(&sc->lock, SA_XLOCKED);
	error = ssam_bat_rq(sc, BAT_CID_BST, p, sizeof(p));
	if (error != 0)
		return (error);
	sc->bst.state = le32dec(&p[0]);
	sc->bst.rate = le32dec(&p[4]);
	sc->bst.cap = le32dec(&p[8]);
	sc->bst.volt = le32dec(&p[12]);
	sc->bst_time = time_uptime;
	sc->bst_valid = 1;
	return (0);
}

/* ACPI battery interface. */
static int
ssam_bat_get_info(device_t dev, void *bix, size_t len)
{
	struct ssam_bat_softc *sc = device_get_softc(dev);
	int error;

	if (len != sizeof(struct acpi_bix) && len != sizeof(struct acpi_bif))
		return (-1);

	sx_xlock(&sc->lock);
	error = 0;
	if (!sc->bix_valid || time_uptime - sc->bix_time >=
	    acpi_battery_get_info_expire())
		error = ssam_bat_update_bix(sc);
	if (error != 0 && !sc->bix_valid) {
		sx_xunlock(&sc->lock);
		return (ENXIO);
	}
	memcpy(bix, &sc->bix, len);
	sx_xunlock(&sc->lock);
	return (0);
}

static int
ssam_bat_get_status(device_t dev, struct acpi_bst *bst)
{
	struct ssam_bat_softc *sc = device_get_softc(dev);

	sx_xlock(&sc->lock);
	if (sc->bst_valid && time_uptime - sc->bst_time < BAT_BST_EXPIRE) {
		*bst = sc->bst;
		sx_xunlock(&sc->lock);
		return (0);
	}
	if (!ssam_bat_present(sc)) {
		sc->bst_valid = 0;
		bst->state = ACPI_BATT_STAT_NOT_PRESENT;
	} else if (ssam_bat_update_bst(sc) == 0) {
		*bst = sc->bst;
	} else {
		bst->state = ACPI_BATT_STAT_NOT_PRESENT;
	}
	sx_xunlock(&sc->lock);
	return (0);
}

/* AC adapter. */
static int
ssam_bat_get_acline(struct ssam_bat_softc *sc, int *acline)
{
	uint8_t rsp[4];
	int error;

	error = ssam_bat_rq(sc, BAT_CID_PSRC, rsp, sizeof(rsp));
	if (error != 0)
		return (error);
	*acline = le32dec(rsp) != 0;
	return (0);
}

static int
ssam_bat_acline_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct ssam_bat_softc *sc = arg1;
	int val;

	if (ssam_bat_get_acline(sc, &val) != 0)
		return (EIO);
	return (sysctl_handle_int(oidp, &val, 0, req));
}

/* Read the AC status and the battery status, and tell userland. */
static void
ssam_bat_poll(void *arg, int pending __unused)
{
	struct ssam_bat_softc *sc = arg;
	uint32_t state;
	int acline;

	/* Do not hide the value of a real ACPI AC adapter. */
	if (sc->acline_added && ssam_bat_get_acline(sc, &acline) == 0 &&
	    acline != sc->acline) {
		sc->acline = acline;
		power_profile_set_state(acline ? POWER_PROFILE_PERFORMANCE :
		    POWER_PROFILE_ECONOMY);
		acpi_UserNotify("ACAD", sc->handle, acline);
	}

	sx_xlock(&sc->lock);
	if (!ssam_bat_present(sc)) {
		sc->bst_valid = 0;
		state = ACPI_BATT_STAT_NOT_PRESENT;
	} else if (ssam_bat_update_bst(sc) == 0)
		state = sc->bst.state & ACPI_BATT_STAT_BST_MASK;
	else
		state = sc->state;
	sx_xunlock(&sc->lock);
	if (state != sc->state) {
		sc->state = state;
		acpi_UserNotify("CMBAT", sc->handle, BAT_NOTIFY_STATUS);
	}

	if (!sc->detaching && sc->poll_interval > 0)
		taskqueue_enqueue_timeout(sc->tq, &sc->poll_task,
		    sc->poll_interval * hz);
}

static int
ssam_bat_poll_interval_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct ssam_bat_softc *sc = arg1;
	int error, val;

	val = sc->poll_interval;
	error = sysctl_handle_int(oidp, &val, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	if (val < 0 || val > 3600)
		return (EINVAL);
	sc->poll_interval = val;
	if (val > 0)
		taskqueue_enqueue_timeout(sc->tq, &sc->poll_task, val * hz);
	else
		taskqueue_cancel_timeout(sc->tq, &sc->poll_task, NULL);
	return (0);
}

static void
ssam_bat_add_acline(struct ssam_bat_softc *sc)
{
	struct acpi_softc *acpi_sc;
	device_t acpi_dev;

	SYSCTL_ADD_PROC(device_get_sysctl_ctx(sc->dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(sc->dev)), OID_AUTO,
	    "acline", CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
	    ssam_bat_acline_sysctl, "I", "AC adapter connected");

	/* Do not hide the value of a real ACPI AC adapter. */
	if (devclass_get_count(devclass_find("acpi_acad")) != 0)
		return;
	acpi_dev = devclass_get_device(devclass_find("acpi"), 0);
	if (acpi_dev == NULL)
		return;
	acpi_sc = device_get_softc(acpi_dev);
	if (acpi_sc == NULL || acpi_sc->acpi_sysctl_tree == NULL)
		return;
	sysctl_ctx_init(&sc->acline_ctx);
	SYSCTL_ADD_PROC(&sc->acline_ctx,
	    SYSCTL_CHILDREN(acpi_sc->acpi_sysctl_tree), OID_AUTO, "acline",
	    CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
	    ssam_bat_acline_sysctl, "I", "AC adapter connected (SSAM)");
	sc->acline_added = 1;
}

/* Device interface. */
static int
ssam_bat_probe(device_t dev)
{
	device_set_desc(dev, "Surface SAM Battery");
	return (BUS_PROBE_DEFAULT);
}

static int
ssam_bat_attach(device_t dev)
{
	struct ssam_bat_softc *sc = device_get_softc(dev);
	int error;

	sc->dev = dev;
	sc->tid = SSAM_TID_SAM;
	sc->iid = 0x01;
	sc->handle = ssam_get_acpi_handle(device_get_parent(dev));
	sc->acline = -1;
	sc->state = ACPI_BATT_STAT_NOT_PRESENT;
	sc->poll_interval = BAT_POLL_INTERVAL;
	sx_init(&sc->lock, "ssam battery");

	if (ssam_bat_present(sc)) {
		sx_xlock(&sc->lock);
		error = ssam_bat_update_bix(sc);
		sx_xunlock(&sc->lock);
		if (error == 0)
			device_printf(dev, "%s %s, design capacity %u %s\n",
			    sc->bix.oeminfo, sc->bix.model, sc->bix.dcap,
			    sc->bix.units == ACPI_BIX_UNITS_MW ? "mWh" :
			    "mAh");
		else
			device_printf(dev, "cannot read _BIX (error %d)\n",
			    error);
	} else
		device_printf(dev, "battery not present\n");

	error = acpi_battery_register(dev);
	if (error != 0) {
		device_printf(dev, "cannot register battery\n");
		sx_destroy(&sc->lock);
		return (error);
	}
	ssam_bat_add_acline(sc);

	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "poll_interval", CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE, sc, 0,
	    ssam_bat_poll_interval_sysctl, "I",
	    "Seconds between status reads for notifications (0 = off)");

	sc->tq = taskqueue_create("ssam_bat", M_WAITOK,
	    taskqueue_thread_enqueue, &sc->tq);
	taskqueue_start_threads(&sc->tq, 1, PWAIT, "%s poll",
	    device_get_nameunit(dev));
	TIMEOUT_TASK_INIT(sc->tq, &sc->poll_task, 0, ssam_bat_poll, sc);
	taskqueue_enqueue_timeout(sc->tq, &sc->poll_task, 0);
	return (0);
}

static int
ssam_bat_detach(device_t dev)
{
	struct ssam_bat_softc *sc = device_get_softc(dev);

	sc->detaching = 1;
	while (taskqueue_cancel_timeout(sc->tq, &sc->poll_task, NULL) != 0)
		taskqueue_drain_timeout(sc->tq, &sc->poll_task);
	taskqueue_free(sc->tq);

	if (sc->acline_added)
		sysctl_ctx_free(&sc->acline_ctx);
	acpi_battery_remove(dev);
	sx_destroy(&sc->lock);
	return (0);
}

static device_method_t ssam_bat_methods[] = {
	DEVMETHOD(device_probe,		ssam_bat_probe),
	DEVMETHOD(device_attach,	ssam_bat_attach),
	DEVMETHOD(device_detach,	ssam_bat_detach),

	DEVMETHOD(acpi_batt_get_info,	ssam_bat_get_info),
	DEVMETHOD(acpi_batt_get_status,	ssam_bat_get_status),

	DEVMETHOD_END
};

static driver_t ssam_bat_driver = {
	"battery",
	ssam_bat_methods,
	sizeof(struct ssam_bat_softc),
};

DRIVER_MODULE(ssam_bat, ssam, ssam_bat_driver, 0, 0);
MODULE_DEPEND(ssam_bat, acpi, 1, 1, 1);
