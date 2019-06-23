#pragma once

class QEI{
public:
    QEI();
    virtual ~QEI(){}
    virtual int read();
    virtual void set(int set_value);
    void reset();
};