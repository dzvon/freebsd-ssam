KMOD=	ssam
SRCS=	ssam.c ssam_bat.c
SRCS+=	acpi_if.h bus_if.h device_if.h pci_if.h opt_acpi.h

SYSDIR?=	${HOME}/ssam-work/src/sys

.include <bsd.kmod.mk>
