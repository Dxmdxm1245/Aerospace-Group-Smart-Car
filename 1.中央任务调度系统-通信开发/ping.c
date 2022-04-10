#include <unistd.h>
#include <stdio.h>
#include <x86_64-linux-gnu/sys/socket.h>
#include <netinet/in.h>
#include <x86_64-linux-gnu/sys/types.h>
#include <stdlib.h>
#include <string.h>


#define SERVER_PORT 5000
#define LENGTH_OF_LISTEN_QUEUE 30
#define BUFFER_SIZE 30

int main() // (int argc, char* argv[])
{
    struct sockaddr_in server_addr;
    int server_socket;
    int opt = 1;

    bzero(&server_addr, sizeof(server_addr)); 
    server_addr.sin_family = AF_INET;
    server_addr.sin_addr.s_addr = htons(INADDR_ANY); 
    server_addr.sin_port = htons(SERVER_PORT);  
    server_socket = socket(PF_INET, SOCK_STREAM, 0);

    if (server_socket < 0)
    {
        printf("Create Socket Failed!\n");
        exit(1);
    }


    // bind a socket
    setsockopt(server_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
    if(bind(server_socket, (struct sockaddr*)&server_addr, sizeof(server_addr)))
    {
        printf("Server Bind Port: %d Failed!\n", SERVER_PORT);
        exit(1);
    }

    if (listen(server_socket, LENGTH_OF_LISTEN_QUEUE))
    {
        printf("Server Listen Failed!\n");
        exit(1);
    }      

    while(1)
    {
        struct sockaddr_in client_addr;
        int client_socket;
        socklen_t length;
        char Buffer[BUFFER_SIZE];

    
        length = sizeof(client_addr);
        client_socket = accept(server_socket, (struct sockaddr*)&client_addr, &length);
        if (client_socket < 0)
        {
            printf("Server Accept Failed!\n");
            break;
        }

   
        while(1)
        {
            bzero(Buffer, BUFFER_SIZE);
            length = recv(client_socket, Buffer, BUFFER_SIZE, 0);
            if (length < 0)
            {
                printf("Server Recieve Data Failed!\n");
                break;
            }           

            if ('q' == Buffer[0])
            {
                printf("Quit!\n");
                break;
            }
	    /*if(Buffer[0]=='1')
            {
                printf("%s",Buffer);
                break;
            }
	    for(int i=0;i<5;i++){
                if(Buffer[i]=='4'){
  		printf("%s ",Buffer);}
      		else
		break;
*/
            
            printf("%s\n", Buffer);
            break;
        }

        close(client_socket);
    }

    close(server_socket);
    return 0;
}

