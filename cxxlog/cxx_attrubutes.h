#ifndef CXX_ATTRUBUTES_H
#define CXX_ATTRUBUTES_H

#include "textstream.h"

class Counter
{
public:
    static TextStream& Output(TextStream& os)
    {
        return os << "Kadr"; //"No. " << ++Count();
    }
};

class Mode
{
public:
    static TextStream& Output(TextStream& os)
    {
        return os << "fsm:";
    }
};


#endif /* CXX_ATTRUBUTES_H */
