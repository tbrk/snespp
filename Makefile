# $Id: Makefile 31 2007-04-16 08:33:29Z root $

KMOD    = snespp
SRCS    = snespp.c sneskbd.c
SRCS    += device_if.h bus_if.h

.include <bsd.kmod.mk>
