/* EE201A Winter 2017 Course Project
 * Author: Vishesh Dokania (vdokania@ucla.edu)
 */

#ifndef OAHELPER_H
#define	OAHELPER_H

#include <vector>
#include <string>
#include "oaDesignDB.h"

using namespace std;
using namespace oa;

class OAHelper
{
public:
    OAHelper();
    virtual ~OAHelper();
	static oaPoint GetTermPosition(oaTerm* term);
	static oaPoint GetAbsoluteInstTermPosition(oaInstTerm* instTerm);
	static void MovePinToPosition(oaInstTerm* instTerm, oaPoint newAbsolutePos);
	static void MovePinByOffset(oaInstTerm* instTerm, oaPoint offset);
    
private:

};

#endif	/* OAHELPER_H */

