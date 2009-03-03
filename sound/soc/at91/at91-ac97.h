/*
 * sound/soc/at91/at91-ac97.h  --  ALSA Soc Audio Layer (platform)
 *
 * Copyright (C) 2008 4D Electronics
 *	http://www.4d-electronics.co.nz
 *	Matthew Dombroski <mdombroski@4d-electronics.co.nz>
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 *  Revision history
 *    17 July 2008	Initial version.
 *    04 March 2009	Port to linux-2.6.28
 *    04 July 2009 Port to linux-2.6.30
 */

#ifndef _AT91_AC97_H
#define _AT91_AC97_H

extern struct snd_soc_dai at91_ac97_dai[];
extern struct snd_ac97_bus_ops at91_ac97_ops;

#endif /* _AT91_AC97_H */

