#include "detection_base.h"

DetectionBase::DetectionBase() :
    is_new_(0),
    is_correct_(0),
    is_associated_(0),
    id_(0),
    time_stamp_()
{
    //
}

DetectionBase::~DetectionBase()
{
    //
}

void DetectionBase::setAsNew()
{
    is_new_ = true;
}
        
void DetectionBase::setAsOld()
{
    is_new_ = false;
}

void DetectionBase::setAsCorrect()
{
    is_correct_ = true; 
}

void DetectionBase::setAsUncorrect()
{
    is_correct_ = false;
}

void DetectionBase::setAsAssociated()
{
    is_associated_ = true;
}

void DetectionBase::setAsUnAssociated()
{
    is_associated_ = false; 
}
        
bool DetectionBase::isNew() const
{
    return is_new_;
}

bool DetectionBase::isCorrect() const
{
    return is_correct_;
}

bool DetectionBase::isAssociated() const
{
    return is_associated_;
}

unsigned int DetectionBase::getId() const
{
    return id_;
}

void DetectionBase::setId(unsigned int _new_id)
{
    id_ = _new_id; 
}
