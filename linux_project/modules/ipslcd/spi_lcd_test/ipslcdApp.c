#include "stdio.h"
#include "unistd.h"
#include "sys/types.h"
#include "sys/stat.h"
#include "sys/ioctl.h"
#include "fcntl.h"
#include "stdlib.h"
#include "string.h"
#include <poll.h>
#include <sys/select.h>
#include <sys/time.h>
#include <signal.h>
#include <fcntl.h>


int main(int argc, char *argv[])
{
    int ret = 0;
    int fd;
    char *filename;

    if (argc != 2) {
		printf("Error Usage!\r\n");
		return -1;
	}

    filename = argv[1];
	fd = open(filename, O_RDWR);
	if(fd < 0) {
		printf("can't open file %s\r\n", filename);
		return -1;
	}
	else
	{
		printf("%s open this ipslcd\r\n");
	}


    ret = close(fd); /* 关闭文件 */
	if(ret < 0){
		printf("file %s close failed!\r\n", filename);
		return -1;
	}



    return 0;
}

