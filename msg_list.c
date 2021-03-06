#include <stdio.h>
#include <stdlib.h>
#include "string.h"
#include "msg_list.h"

message_list* add_message(message_list* lst, char* msg)
{   
    message_list* newer;
    newer = (message_list*) malloc (sizeof (message_list));
    strcpy(newer->message, msg);
    newer->next = NULL;
    newer->last = newer;
    if (lst == NULL)
    {
        lst = newer;
    }
    else
    {
        lst->last->next = newer;
        lst->last = newer;
    }
    return lst;
}

message_list* remove_message(message_list* lst, char* msg)
{
    message_list* old;
    old = lst;
    strcpy(msg, old->message);
    if (lst->next != NULL)
    {
        lst = lst->next;
        lst->last = old->last;
    }
    else
    {
        lst = NULL;
    }
    free(old);
    return lst;
}



