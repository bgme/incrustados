#include "Task.hpp"

m_u8TaskID = 0;

Task::Task()
{
    m_u8TaskID = m_u8NextTaskID;
    m_u8NextTaskID++;
    m_bIsFinished = false;
}


