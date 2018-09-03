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
 * $Id: sneskbd.c 31 2007-04-16 08:33:29Z root $
 */

/* These routines were copied from Maksim Yevmenkin's vkbd driver.
 * Interfacing with the vkbd driver from another driver (i.e. from within
 * the kernel) is non-trivial.
 */

#include <sys/cdefs.h>

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/systm.h>
#include <sys/conf.h>		/* cdevsw stuff */
#include <sys/kernel.h>		/* SYSINIT stuff */
#include <sys/uio.h>		/* SYSINIT stuff */
#include <sys/malloc.h>

#include <sys/queue.h>
#include <sys/taskqueue.h>

#include <sys/kbio.h>
#include <dev/kbd/kbdreg.h>
#include <dev/kbd/kbdtables.h>

#include "snespp.h"

MALLOC_DECLARE(M_SNES);
MALLOC_DEFINE(M_SNES, SNESPP_NAME, "SNES Control Pad");

void snesk_read_task(void *xkbd, int pending);

static int		snesk_configure(int flags);
static kbd_probe_t	snesk_probe;
static kbd_init_t	snesk_init;
static kbd_term_t	snesk_term;
static kbd_intr_t	snesk_intr;
static kbd_test_if_t	snesk_test_if;
static kbd_enable_t	snesk_enable;
static kbd_disable_t	snesk_disable;
static kbd_read_t	snesk_read;
static kbd_check_t	snesk_check;
static kbd_read_char_t	snesk_read_char;
static kbd_check_char_t	snesk_check_char;
static kbd_ioctl_t	snesk_ioctl;
static kbd_lock_t	snesk_lock;
static kbd_clear_state_t snesk_clear_state;
static kbd_get_state_t	snesk_get_state;
static kbd_set_state_t	snesk_set_state;
static kbd_poll_mode_t	snesk_poll;

keyboard_switch_t snesksw = {
	snesk_probe,
	snesk_init,
	snesk_term,
	snesk_intr,
	snesk_test_if,
	snesk_enable,
	snesk_disable,
	snesk_read,
	snesk_check,
	snesk_read_char,
	snesk_check_char,
	snesk_ioctl,
	snesk_lock,
	snesk_clear_state,
	snesk_get_state,
	snesk_set_state,
	genkbd_get_fkeystr,
	snesk_poll,
	genkbd_diag,
};

struct snesk_state {
	struct cdev *	ks_dev;		/* control device */
	int		ks_mode;	/* keyboard mode:
					    KS_XLATE	return ASCII
					    KS_RAW	return scancodes
					    KS_CODE	return keycodes */
	
	struct snespp_softc *sc;
};

typedef struct snesk_state snesk_state_t;

KEYBOARD_DRIVER(snespp, snesksw, snesk_configure);

void
snesk_read_task(void *xkbd, int pending)
{
	keyboard_t	*kbd = (keyboard_t *) xkbd;
	snesk_state_t	*state = (snesk_state_t *) kbd->kb_data;

	(*kbdsw[kbd->kb_index]->intr)(kbd, NULL);
	snespp_clearpending(state->sc);
}

int
snesk_create(struct snespp_softc *sc)
{
	int			unit = device_get_unit(sc->device);
	int			error = -1;
	keyboard_switch_t	*sw = NULL;
	keyboard_t		*kbd = NULL;

	/* Add keyboard functionality */
	kbd_add_driver(&snespp_kbd_driver);
	if ((sw = kbd_get_switch(SNESPP_NAME)) == NULL)
		goto errexit;

	if ((error = (*sw->probe)(unit, NULL, 0)) != 0 ||
	    (error = (*sw->init)(unit, &kbd, (void *)sc, 0)) != 0)
		goto errexit;

	TASK_INIT(&sc->rd_task, 0, snesk_read_task, (void *)kbd);
	sc->run_rd_task = 1;
	
/*	if ((error = (*sw->enable)(kbd)) != 0) {
		(*sw->term)(kbd);
		goto errexit;
	}*/

#ifdef KBD_INSTALL_CDEV
	if ((error = kbd_attach(kbd)) != 0) {
		(*sw->disable)(kbd);
		(*sw->term)(kbd);
		goto errexit;
	}
#endif

	/* see snesk_disable... */
	snespp_requestpolling(sc, 1);
	
	return (0);

errexit:
	return (error);
}

int
snesk_destroy(struct snespp_softc *sc)
{
	keyboard_switch_t	*sw = NULL;
	keyboard_t		*kbd;

	/* see snesk_disable... */
	snespp_requestpolling(sc, 0);

	sc->run_rd_task = 0;

	kbd = kbd_get_keyboard(kbd_find_keyboard(SNESPP_NAME,
						 dev2unit(sc->dev)));

	if ((sw = kbd_get_switch(SNESPP_NAME)) != NULL) {
		(*sw->disable)(kbd);
#ifdef KBD_INSTALL_CDEV
		kbd_detach(kbd);
#endif
		(*sw->term)(kbd);
	}
	kbd_delete_driver(&snespp_kbd_driver);

	return (0);
}

static int
snesk_configure(int flags)
{
	return (0);
}

static int
snesk_probe(int unit, void *arg, int flags)
{
	return (0);
}

static int
snesk_init(int unit, keyboard_t **kbdp, void *arg, int flags)
{
	keyboard_t		*kbd = NULL;
	snesk_state_t		*state = NULL;
	keymap_t		*keymap = NULL;
	accentmap_t		*accmap = NULL;
	fkeytab_t		*fkeymap = NULL;
	int		 	fkeymap_size;
	int		 	error, needfree;
	struct snespp_softc*	sc = (struct snespp_softc *)arg;

	if (*kbdp == NULL) {
		*kbdp = kbd = malloc(sizeof(*kbd), M_SNES, M_NOWAIT | M_ZERO);
		state = malloc(sizeof(*state), M_SNES, M_NOWAIT | M_ZERO);
		keymap = malloc(sizeof(key_map), M_SNES, M_NOWAIT);
		accmap = malloc(sizeof(accent_map), M_SNES, M_NOWAIT);
		fkeymap = malloc(sizeof(fkey_tab), M_SNES, M_NOWAIT);
		fkeymap_size = sizeof(fkey_tab)/sizeof(fkey_tab[0]);
		needfree = 1;
		if ((kbd == NULL) || (state == NULL) || (keymap == NULL) ||
			(accmap == NULL) || (fkeymap == NULL)) {
			error = ENOMEM;
			goto bad;
		}

	} else if (KBD_IS_INITIALIZED(*kbdp) && KBD_IS_CONFIGURED(*kbdp)) {
		return (0);
	} else {
		kbd = *kbdp;
		state = (snesk_state_t *) kbd->kb_data;
		keymap = kbd->kb_keymap;
		accmap = kbd->kb_accentmap;
		fkeymap = kbd->kb_fkeytab;
		fkeymap_size = kbd->kb_fkeytab_size;
		needfree = 0;
	}

	if (!KBD_IS_PROBED(kbd)) {
		kbd_init_struct(kbd, SNESPP_NAME, KB_OTHER, unit, flags, 0, 0);
		bcopy(&key_map, keymap, sizeof(key_map));
		bcopy(&accent_map, accmap, sizeof(accent_map));
		bcopy(fkey_tab, fkeymap,
			imin(fkeymap_size*sizeof(fkeymap[0]), sizeof(fkey_tab)));
		kbd_set_maps(kbd, keymap, accmap, fkeymap, fkeymap_size);

		kbd->kb_data = (void *)state;
		state->sc = sc;
	
		KBD_FOUND_DEVICE(kbd);
		KBD_PROBE_DONE(kbd);
	}
	if (!KBD_IS_INITIALIZED(kbd) && !(flags & KB_CONF_PROBE_ONLY)) {
		kbd->kb_config = flags & ~KB_CONF_PROBE_ONLY;
		KBD_INIT_DONE(kbd);
	}
	if (!KBD_IS_CONFIGURED(kbd)) {
		if (kbd_register(kbd) < 0) {
			error = ENXIO;
			goto bad;
		}
		KBD_CONFIG_DONE(kbd);
	}

	return (0);
bad:
	if (needfree) {
		if (state != NULL)
			free(state, M_SNES);
		if (keymap != NULL)
			free(keymap, M_SNES);
		if (accmap != NULL)
			free(accmap, M_SNES);
		if (fkeymap != NULL)
			free(fkeymap, M_SNES);
		if (kbd != NULL) {
			free(kbd, M_SNES);
			*kbdp = NULL;
		}
	}
	return (error);
}

static int
snesk_term(keyboard_t *kbd)
{
	snesk_state_t	*state = (snesk_state_t *) kbd->kb_data;

	kbd_unregister(kbd);

	bzero(state, sizeof(*state));
	free(state, M_SNES);

	free(kbd->kb_keymap, M_SNES);
	free(kbd->kb_accentmap, M_SNES);
	free(kbd->kb_fkeytab, M_SNES);
	free(kbd, M_SNES);

	return (0);
}

static int
snesk_intr(keyboard_t *kbd, void *arg)
{
	snesk_state_t	*state = (snesk_state_t *) kbd->kb_data;

	if (KBD_IS_ACTIVE(kbd) && KBD_IS_BUSY(kbd)) {
		(*kbd->kb_callback.kc_func)(kbd, KBDIO_KEYINPUT,
					    kbd->kb_callback.kc_arg);
	} else {
		snespp_queue_clear(state->sc);
	}

	return (0);
}
static int
snesk_test_if(keyboard_t *kbd)
{
	return (0);
}

static int
snesk_enable(keyboard_t *kbd)
{
	/*snesk_state_t	*state = (snesk_state_t *) kbd->kb_data;*/

	KBD_ACTIVATE(kbd);
	/*
	snespp_requestpolling(state->sc, KBD_IS_ACTIVE(kbd));
	*/

	return (0);
}

static int
snesk_disable(keyboard_t *kbd)
{
	/*snesk_state_t	*state = (snesk_state_t *) kbd->kb_data;*/

	KBD_DEACTIVATE(kbd);
	/*
	 * This call should stop the updating thread when
	 * the keyboard has been detached. On FreeBSD-6.2
	 * it seems that the command:
	 *    kbdcontrol -A snes0 < /dev/console
	 * does not trigger a call to snesk_disable, whereas:
	 *    kbdcontrol -a snes0 < /dev/console
	 * does. This breaks the counting mechanism used by
	 * the KBD_IS_ACTIVE, KBD_ACTIVATE and KBD_DEACTIVATE
	 * macros.
	 * To re-enable this feature, uncomment the line inside
	 * snesk_enable and the line below and comment out the
	 * calls to snespp_requestpolling in snesk_create and
	 * snesk_destroy.
	 *
	snespp_requestpolling(state->sc, KBD_IS_ACTIVE(kbd));
	*/

	return (0);
}

/* Read one byte (scan code) from keyboard */
static int
snesk_read(keyboard_t *kbd, int wait)
{
	snesk_state_t	*state = (snesk_state_t *) kbd->kb_data;
	int		c = snespp_queue_getc(state->sc);

	return ((c > 0)?c:NOKEY);
}

static int
snesk_check(keyboard_t *kbd)
{
	snesk_state_t	*state = (snesk_state_t *) kbd->kb_data;

	if (!KBD_IS_ACTIVE(kbd))
		return (FALSE);
	
	return (!snespp_queue_empty(state->sc));
}

/* read a scancode, turn it into a keycode (if not RAW) then return */
static u_int
snesk_read_char(keyboard_t *kbd, int wait)
{
	snesk_state_t	*state = (snesk_state_t *) kbd->kb_data;
	int		c = snespp_queue_getc(state->sc);

	return ((c > 0)?c:NOKEY);
}

static int
snesk_check_char(keyboard_t *kbd)
{
	snesk_state_t	*state = (snesk_state_t *) kbd->kb_data;

	if (!KBD_IS_ACTIVE(kbd))
		return (FALSE);
	
	return (!snespp_queue_empty(state->sc));
}

static int
snesk_ioctl(keyboard_t *kbd,  u_long cmd, caddr_t arg)
{
	snesk_state_t	*state = (snesk_state_t *) kbd->kb_data;

	switch (cmd) {
	case KDGKBMODE:
		*(int *)arg = state->ks_mode;
		break;

	case KDSKBMODE:
		switch (*(int *)arg) {
		case K_XLATE:
		case K_RAW:
		case K_CODE:
			state->ks_mode = *(int *)arg;
			break;

		default:
			return (EINVAL);
		}
		break;
	
	case KDGKBSTATE:
		*(int *)arg = 0;
		break;
	
	case KDSKBSTATE:
		break;
	
	default:
		return (genkbd_commonioctl(kbd, cmd, arg));
	}

	return (0);
}

static int
snesk_lock(keyboard_t *kbd, int lock)
{
	return (1);
}

static void
snesk_clear_state(keyboard_t *kbd)
{
}

static int
snesk_get_state(keyboard_t *kbd, void *buf, size_t len)
{
	if (len == 0)
		return (sizeof(snesk_state_t));
	if (len < sizeof(snesk_state_t))
		return (-1);
	bcopy(kbd->kb_data, buf, sizeof(snesk_state_t));

	return 0;
}

static int
snesk_set_state(keyboard_t *kbd, void *buf, size_t len)
{
	if (len < sizeof(snesk_state_t))
		return (ENOMEM);
	bcopy(buf, kbd->kb_data, sizeof(snesk_state_t));
	return (0);
}

static int
snesk_poll(keyboard_t *kbd, int on)
{
	return (0);
}

