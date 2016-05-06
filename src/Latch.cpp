/*
 * Latch.cpp
 *
 *  Created on: Mar 15, 2016
 *      Author: johnr
 */

#include <Latch.h>

Latch::Latch()
{
	// TODO Auto-generated constructor stub

}

void Latch::Toggle(bool b, bool &bp, bool &tb)
{
	if (b && !bp) {
		bp = true;
		tb ? tb = false : tb = true;
	} else if (!b)
		bp = false;
}
