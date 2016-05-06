/*
 * Latch.h
 *
 *  Created on: Mar 15, 2016
 *      Author: johnr
 */

#ifndef SRC_LATCH_H_
#define SRC_LATCH_H_

class Latch {

public:
	Latch();
	void Toggle(bool b, bool &bp, bool &tb);
};

#endif /* SRC_LATCH_H_ */
