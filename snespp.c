/*-
 * Copyright (c) 2007 Timothy Bourke.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * $Id: snespp.c 31 2007-04-16 08:33:29Z root $
 */

#include <sys/cdefs.h>

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/fcntl.h>
#include <sys/kernel.h>
#include <sys/kthread.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/namei.h>
#include <sys/proc.h>
#include <sys/uio.h>

#include <sys/malloc.h>
#include <sys/queue.h>
#include <sys/taskqueue.h>

#include <machine/bus.h>
#include <machine/resource.h>
#include <sys/rman.h>

#include <dev/ppbus/ppbconf.h>
#include <dev/ppbus/ppb_msq.h>

#include <sys/joystick.h>

#include "snespp.h"

#define DEVICETOSOFTC(d) ((struct snespp_softc *)device_get_softc(d));
#define UNITODEVICE(unit) (devclass_get_device(snespp_devclass, (unit)))
#define UNITOSOFTC(unit) \
	((struct snespp_softc *)devclass_get_softc(snespp_devclass, (unit)))

#define INCQ(x) (x = (x + 1) % SNESPP_EVENTQ_SIZE)

#define SNESPP_MAKE	0x0000
#define SNESPP_BREAK	0x0080
static unsigned int snespp_events[5][SNESPP_NUM_BUTTONS] = { /* AT Codes */
  /*   B     Y   SEL  START   UP  DOWN  LEFT  RIGHT    A     X     L     R */
  /*   b     y     p     o     q     w     e     u     a     x     l     r */
  { 0x30, 0x15, 0x19, 0x18, 0x10, 0x11, 0x12, 0x16, 0x1e, 0x2d, 0x26, 0x13 },
  /*   z     c     s  ENTR   KP8   KP2   KP4   KP6   KP+   KP0     1     2 */
  { 0x2c, 0x2e, 0x1f, 0x1c, 0x48, 0x50, 0x4b, 0x4d, 0x4e, 0x52, 0x02, 0x03 },
  /*   t     y     u     i     o     [     ]     \     d     f     g     h */
  { 0x14, 0x15, 0x16, 0x17, 0x18, 0x1a, 0x1b, 0x2b, 0x20, 0x21, 0x22, 0x23 },
  /*   3     4     5     6     7     8     9     0     -     =  BKSP     j */
  { 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x24 },
  /*  F1    F2    F3    F4    F5    F6    F7    F8    F9   F10   F11   F12 */
  { 0x3b, 0x3c, 0x3d, 0x3e, 0x3f, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46 },
};

int snespp_startpolling(struct snespp_softc *);
int snespp_stoppolling(struct snespp_softc *);
static void snespp_updateeventq(void *);
static struct proc *snespp_eventq_proc;

int
snespp_queue_getc(struct snespp_softc *sc)
{
	int	c;

	mtx_lock(&sc->eventq_lock);

	if (sc->eq_tail == sc->eq_head) {
		mtx_unlock(&sc->eventq_lock);
		return (-1);
	}

	c = sc->eventq[sc->eq_tail];
	INCQ(sc->eq_tail);

	mtx_unlock(&sc->eventq_lock);

	return (c);
}

int
snespp_queue_empty(struct snespp_softc *sc)
{
	/* No locking. The buffer status could as easily change when we
	 * release the lock as between the read of eq_head and that of
	 * eq_tail. */
	return (sc->eq_head == sc->eq_tail);
}

void
snespp_queue_clear(struct snespp_softc *sc)
{
	/* No locking as it does not buy anything. */
	sc->eq_tail = sc->eq_head;
}

void
snespp_clearpending(struct snespp_softc *sc)
{
	mtx_lock(&sc->eventq_lock);
	sc->state &= ~TASKPENDING;
	wakeup(&sc->rd_task);
	mtx_unlock(&sc->eventq_lock);
}

void
snespp_requestpolling(struct snespp_softc *sc, int poll)
{
	mtx_lock(&sc->eventq_lock);

	if (poll && !(sc->state & POLLINGREQUESTED)) {
		snespp_startpolling(sc);
		sc->state &= ~POLLINGREQUESTED;

	} else if (!poll && (sc->state & POLLINGREQUESTED)) {
		snespp_stoppolling(sc);
		sc->state |= POLLINGREQUESTED;
	}

	mtx_unlock(&sc->eventq_lock);
}

/****************************************************************************
 **                           hardware interface
 ****************************************************************************/

static int
snespp_request_ppbus(device_t dev, int how)
{
	device_t ppbus = device_get_parent(dev);
	struct snespp_softc *sc = DEVICETOSOFTC(dev);
	int error;

	if (sc->state & HAVEBUS)
		return (0);

	if ((error = ppb_request_bus(ppbus, dev, how)) == 0)
		sc->state |= HAVEBUS;

	return error;
}

static int
snespp_release_ppbus(device_t dev)
{
	device_t ppbus = device_get_parent(dev);
	struct snespp_softc *sc = DEVICETOSOFTC(dev);
	int error;

	if ((error = ppb_release_bus(ppbus, dev)) == 0)
		sc->state &= ~HAVEBUS;

	return (error);
}

/*
 * Poll the parallel port using the serial protocol for Nintendo control
 * pads. Documented widely on The Internet, but specifically a post by
 * Jim Christy to Sci.Electronics entitled:
 *     `Super Nintendo Entertainment System: pinouts & protocol'
 *
 * The microseq(9) features of ppbus and ppc are used. There is a small
 * benefit in maximising the adjacency of DELAY MS_OP_DELAY and
 * MS_OP_RASSERT instructions.
 *
 * This sequence takes ((length + 1) * 12) uSec with spinning waits. If
 * executed at 60Hz with a length of 12 that is about 0.25% CPU Utilization
 * (obviously regardless of processor speed).
 */
static void
snespp_updatebuffer(struct snespp_softc *sc)
{
	device_t ppbus = device_get_parent(sc->device);

	struct ppb_microseq poll_microseq[] = {
		#define INPUT_BUFFER    MS_PARAM(0, 0, MS_TYP_PTR)
		#define INPUT_LENGTH    MS_PARAM(1, 0, MS_TYP_INT)

		MS_PTR(MS_UNKNOWN),
		MS_SET(MS_UNKNOWN),
		/* 12 uSec latch pulse */
		MS_DASS(SNESD_POWER | SNESD_CLOCK | SNESD_LATCH),
		MS_DELAY(6),
     /*loop:*/  MS_DELAY(6),
		MS_DASS(SNESD_POWER | SNESD_CLOCK),	/* clk: low->high */
		MS_DELAY(6),
		MS_DASS(SNESD_POWER),			/* clk: high->low */
		MS_RFETCH_P(1, MS_REG_STR, MS_FETCH_ALL),/* sample */
		MS_DBRA(-6),				/* goto loop */
		MS_RET(0),
	};

	ppb_MS_init_msq(poll_microseq, 2,
			INPUT_BUFFER, sc->buffer[sc->curbuffer],
			INPUT_LENGTH, SNESPP_NUM_BUTTONS);
	ppb_MS_microseq(ppbus, sc->device, poll_microseq, NULL);
}

/* kthread to poll the controllers.
 * Button down and up events are added to the eventq. */
static void
snespp_updateeventq(void *arg)
{
	struct snespp_softc	*sc = (struct snespp_softc *)arg;
	int			i, error, eq_head;
	int			clk = 0;
	char			status;
	char			*buffer;

	if ((error = snespp_request_ppbus(sc->device, PPB_DONTWAIT)) != 0)
		kthread_exit(error);

	snespp_updatebuffer(sc);

    for (;;) {
	tsleep(curthread->td_proc, PWAIT | PCATCH, SNESPP_NAME "wait",
	       (hz / SNESPP_POLL_HZ) + 1);

	kthread_suspend_check(snespp_eventq_proc);
	if (!sc->eventq_active)
		break;

	sc->curbuffer = (sc->curbuffer + 1) % 2;
	snespp_updatebuffer(sc);

	mtx_lock(&sc->eventq_lock);

	buffer = sc->buffer[sc->curbuffer];
	eq_head = sc->eq_head;

	for (i=0; i<SNESPP_NUM_BUTTONS; ++i) {
		if ((status = (char)(sc->buffer[0][i] ^ sc->buffer[1][i])) == 0)
			continue;

		if (status & SNESPP_PAD0) {
			sc->eventq[eq_head] = snespp_events[0][i] |
			    ((buffer[i] & SNESPP_PAD0)?
				SNESPP_BREAK:SNESPP_MAKE);
			INCQ(eq_head);
		}
#if (SNESPP_NUM_CONTROLLERS >= 2)
		if (status & SNESPP_PAD1) {
			sc->eventq[eq_head] = snespp_events[1][i] |
			    ((buffer[i] & SNESPP_PAD1)?
				SNESPP_BREAK:SNESPP_MAKE);
			INCQ(eq_head);
		}
#endif
#if (SNESPP_NUM_CONTROLLERS >= 3)
		if (status & SNESPP_PAD2) {
			sc->eventq[eq_head] = snespp_events[2][i] |
			    ((buffer[i] & SNESPP_PAD2)?
				SNESPP_BREAK:SNESPP_MAKE);
			INCQ(eq_head);
		}
#endif
#if (SNESPP_NUM_CONTROLLERS >= 2)
		if (status & SNESPP_PAD3) {
			sc->eventq[eq_head] = snespp_events[3][i] |
			    ((buffer[i] & SNESPP_PAD3)?
				SNESPP_BREAK:SNESPP_MAKE);
			INCQ(eq_head);
		}
#endif
#if (SNESPP_NUM_CONTROLLERS >= 4)
		if (status & SNESPP_PAD4) {
			sc->eventq[eq_head] = snespp_events[4][i] |
			    ((buffer[i] & SNESPP_PAD4)?
				SNESPP_BREAK:SNESPP_MAKE);
			INCQ(eq_head);
		}
#endif
#if (SNESPP_NUM_CONTROLLERS >= 5)
		if (status & SNESPP_PAD5) {
			sc->eventq[eq_head] = snespp_events[5][i] |
			    ((buffer[i] & SNESPP_PAD5)?
				SNESPP_BREAK:SNESPP_MAKE);
			INCQ(eq_head);
		}
#endif
	}

	sc->eq_head = eq_head;

	clk++;
	if ((clk % SNESPP_TASK_FREQ == 0) && (sc->eq_head != sc->eq_tail)) {
		if ((sc->run_rd_task) &&
		    !(sc->state & TASKPENDING) &&
		    taskqueue_enqueue(taskqueue_swi, &sc->rd_task) == 0)
			sc->state |= TASKPENDING;
	}
	mtx_unlock(&sc->eventq_lock);
	
    } /* main loop */

	snespp_release_ppbus(sc->device);
	kthread_exit(0);
}

int
snespp_startpolling(struct snespp_softc *sc)
{
	int error;

	if (sc->eventq_active == 1)
		return 0;

	sc->eventq_active = 1;
	error = kthread_create(snespp_updateeventq, (void *)sc,
			       &snespp_eventq_proc, 0, 0, SNESPP_NAME "eventq");
	
	return (error);
}

int
snespp_stoppolling(struct snespp_softc *sc)
{
	int result;

	if (sc->eventq_active == 0)
		return 0;

	sc->eventq_active = 0;
	result = tsleep(snespp_eventq_proc, PWAIT | PCATCH,
			SNESPP_NAME " wait eventq term",
				    (3 * hz) / SNESPP_POLL_HZ);
	
	if (result == EWOULDBLOCK)
		device_printf(sc->device, "timeout on eventq kthread.\n");
	
	return (result);
}

/****************************************************************************
 **                           character device
 ****************************************************************************/

static d_open_t		snesopen;
static d_close_t	snesclose;
static d_read_t		snesread;
static d_ioctl_t	snesioctl;

static struct cdevsw snespp_cdevsw = {
	.d_version =	D_VERSION,
	.d_open =	snesopen,
	.d_close =	snesclose,
	.d_read =	snesread,
	.d_ioctl =	snesioctl,
	.d_name =	SNESPP_NAME,
};

static devclass_t snespp_devclass;

static int
snesioctl (struct cdev *dev, u_long cmd, caddr_t data, int flag, struct thread *td)
{
	u_int unit = minor(dev);
	struct snespp_softc *sc = UNITOSOFTC(unit);

	(void)sc; /* Delete this line after using sc. */
	switch (cmd) {
	case JOY_SETTIMEOUT:
	case JOY_GETTIMEOUT:
	case JOY_SET_X_OFFSET:
	case JOY_SET_Y_OFFSET:
	case JOY_GET_X_OFFSET:
	case JOY_GET_Y_OFFSET:
		return (0);

	default:
		return (ENXIO);
	}

	return (0);
}

static int
snesopen(struct cdev *dev, int oflags, int devtype, struct thread *td)
{
	u_int unit = minor(dev);
	struct snespp_softc *sc = UNITOSOFTC(unit);
	int error;

	sc->curbuffer = 0;

	mtx_lock(&sc->eventq_lock);
	sc->eq_head = sc->eq_tail = 0;
	mtx_unlock(&sc->eventq_lock);

	snespp_stoppolling(sc);
	sc->state |= SNESDOPEN;

	if ((error = snespp_request_ppbus(sc->device, PPB_DONTWAIT)) != 0)
		return error;

	return (0);
}

static int
snesclose(struct cdev *dev, int fflag, int devtype, struct thread *td)
{
	u_int unit = minor(dev);
	struct snespp_softc *sc = UNITOSOFTC(unit);

	mtx_lock(&sc->eventq_lock);
	mtx_unlock(&sc->eventq_lock);

	snespp_release_ppbus(sc->device);
	sc->state &= ~SNESDOPEN;
	if (sc->state | POLLINGREQUESTED)
		snespp_startpolling(sc);

	return (0);
}

static int
snesread(struct cdev *dev, struct uio *uio, int ioflag)
{
	u_int			unit = minor(dev);
	struct snespp_softc	*sc = UNITOSOFTC(unit);
	struct joystick		jdata;
	int			toread, rv = 0;
	char			*buffer = sc->buffer[sc->curbuffer];

	if ((uio->uio_resid <= 0) || (uio->uio_rw != UIO_READ))
		return (EINVAL);

	mtx_lock(&sc->eventq_lock);

	snespp_updatebuffer(sc);

	jdata.x = jdata.y = SNESPP_JOY_MID;
	if ((buffer[SNESPP_LEFT] & SNESPP_PAD0) == 0) {
		jdata.x = SNESPP_JOY_MIN;
	} else if ((buffer[SNESPP_RIGHT] & SNESPP_PAD0) == 0) {
		jdata.x = SNESPP_JOY_MAX;
	}

	if ((buffer[SNESPP_UP] & SNESPP_PAD0) == 0) {
		jdata.y = SNESPP_JOY_MIN;
	} else if ((buffer[SNESPP_DOWN] & SNESPP_PAD0) == 0) {
		jdata.y = SNESPP_JOY_MAX;
	}

	jdata.b1 = (buffer[SNESPP_Y] & SNESPP_PAD0)?0:1;
	jdata.b2 = (buffer[SNESPP_B] & SNESPP_PAD0)?0:1;

	toread = min(uio->uio_resid, sizeof(jdata));
	uio->uio_offset = 0;
	rv = uiomove((void *)&jdata, toread, uio);

	mtx_unlock(&sc->eventq_lock);

	return (rv);
}

/****************************************************************************
 **                            newbus driver (ppbus)
 ****************************************************************************/

static int snespp_attach(device_t device);
static int snespp_detach(device_t device);

static void
snespp_identify(driver_t *driver, device_t parent)
{
	device_t dev;

	dev = device_find_child(parent, SNESPP_NAME, 0);
	if (!dev)
	BUS_ADD_CHILD(parent, 0, SNESPP_NAME, -1);
}

static int
snespp_probe(device_t dev)
{
	device_set_desc(dev, "SNES Controller");

	return (0);
}

static int
snespp_attach(device_t device)
{
	struct snespp_softc	*sc = DEVICETOSOFTC(device);
	int			unit = device_get_unit(device);

	sc->state = 0;
	sc->curbuffer = 0;
	sc->device = device;
	sc->run_rd_task = 0;

	sc->eq_head = sc->eq_tail = 0;
	mtx_init(&sc->eventq_lock, SNESPP_NAME " eventq lock", NULL, MTX_DEF);

	sc->dev = make_dev(&snespp_cdevsw, unit,
			   UID_ROOT, GID_OPERATOR,
			   0600, SNESPP_NAME "%d", unit);
	
	snesk_create(sc);

	return (0);
}

static int
snespp_detach(device_t device)
{
	struct snespp_softc 	*sc = DEVICETOSOFTC(device);

	snespp_stoppolling(sc);
	
	while (sc->state & TASKPENDING)
		tsleep(&sc->rd_task, PWAIT | PCATCH, SNESPP_NAME "d", 0);

	snesk_destroy(sc);

	if (sc->dev)
		destroy_dev(sc->dev);

	mtx_destroy(&sc->eventq_lock);

	return (0);
}

static device_method_t snespp_methods[] = {
	/* Device interface */
	DEVMETHOD(device_identify,	snespp_identify),
	DEVMETHOD(device_probe,		snespp_probe),
	DEVMETHOD(device_attach,	snespp_attach),
	DEVMETHOD(device_detach,	snespp_detach),
	{ 0, 0 }
};

static driver_t snespp_driver = {
	SNESPP_NAME,
	snespp_methods,
	sizeof(struct snespp_softc),
};

DRIVER_MODULE(snes, ppbus, snespp_driver, snespp_devclass, 0, 0);
MODULE_DEPEND(snes, ppbus, 1, 1, 1);
