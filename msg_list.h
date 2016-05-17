
typedef struct node{
    char message[256];
    struct node* next;
    struct node* last;
}message_list;

message_list* add_message (message_list* lst, char* msg);

message_list* remove_message (message_list* lst, char* msg);

