#include "Scheduler.hpp"

// - Scheduler constructor
Scheduler::Scheduler()
{
    m_u8OpenSlots = static_cast<uint8_t>(NUMBER_OF_SLOTS);
    m_u8NextSlot = 0;

    for (int index = 0; index < NUMBER_OF_SLOTS; index++)
    {
        m_aSchedule[index].pToAttach = (uintptr_t) 0; // Init to an invalid pointer
        run_flag[index] = false; // Init without run anything

        for (int j = 0; j < MAIL_SIZE; j++)
        {
            mail_box[index][j] = 0U;
        }
    }
    return;
}
// - The attach function, inserts the task into the schedule slots.
uint8_t Scheduler::attach(Task * i_ToAttach, uint64_t i_u64TickInterval,
                          uint32_t ** o_u32MailBox, bool **run_flag)
{
    uint8_t l_ErrorCode = NO_ERR;
    st_TaskInfo l_st_StructToAttach;

    l_st_StructToAttach.pToAttach = i_ToAttach;
    l_st_StructToAttach.u64ticks = this->m_u64ticks;
    l_st_StructToAttach.u64TickInterval = 0;
    l_st_StructToAttach.u64TickIntervalInitValue = i_u64TickInterval;

    if ((m_u8OpenSlots > 0) && (m_u8NextSlot < NUMBER_OF_SLOTS))
    {
        m_aSchedule[m_u8NextSlot] = l_st_StructToAttach;
        m_u8OpenSlots--;
        m_u8NextSlot++;

        *run_flag = &this->run_flag[0];
        *o_u32MailBox = &this->mail_box[0][0];
    }
    else
    {
        l_ErrorCode = RET_ERR;
    }
    return l_ErrorCode;
}
// - Execute the current schedule
uint8_t Scheduler::run(void)
{
    int l_iNextTaskSlot = 0U;
    Task * l_pNextTask = (uintptr_t) 0;
    uint8_t l_u8ReturnCode = NO_ERR;

    while (l_iNextTaskSlot < NUMBER_OF_SLOTS)
    {
        l_pNextTask =
                static_cast<Task *>(m_aSchedule[l_iNextTaskSlot].pToAttach);
        if (l_pNextTask != ((uintptr_t) 0))
        {
            if (m_aSchedule[l_iNextTaskSlot].u64TickIntervalInitValue != 0)
            {
                if (m_aSchedule[l_iNextTaskSlot].u64TickInterval == 0)
                {
                    l_pNextTask->run();
                }
                m_aSchedule[l_iNextTaskSlot].u64TickInterval++;

                if (m_aSchedule[l_iNextTaskSlot].u64TickInterval
                        > m_aSchedule[l_iNextTaskSlot].u64TickIntervalInitValue)
                {
                    m_aSchedule[l_iNextTaskSlot].u64TickInterval = 0;
                }
            }
            else if (this->run_flag[m_aSchedule[l_iNextTaskSlot].pToAttach->m_u8TaskID])
            {
                l_pNextTask->run();
                this->run_flag[m_aSchedule[l_iNextTaskSlot].pToAttach->m_u8TaskID] =
                        false; // Clear the event flag
            }
        }
        l_iNextTaskSlot++;
    }
    CalculateNextSchedule(); // TODO

    return l_u8ReturnCode;
}
// - Execute the setup function for all tasks
uint8_t Scheduler::setup(void)
{
    int l_iNextTaskSlot = 0U;
    Task * l_pNextTask = (uintptr_t) 0;
    uint8_t l_u8ReturnCode = NO_ERR;

    while (l_iNextTaskSlot < NUMBER_OF_SLOTS)
    {
        l_pNextTask =
                static_cast<Task *>(m_aSchedule[l_iNextTaskSlot].pToAttach);
        if (l_pNextTask != ((uintptr_t) 0))
        {
            l_pNextTask->setup();
        }
        l_iNextTaskSlot++;
    }
    return l_u8ReturnCode;
}

uint8_t Scheduler::CalculateNextSchedule(void)
{
    return (NO_ERR);
}
uint8_t Scheduler::SortScheduleByPriority(Task * i_pSchedule)
{
    return (NO_ERR);
}
