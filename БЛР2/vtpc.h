// vtpc.h
#ifndef VTPC_H
#define VTPC_H

#include <sys/types.h>

// Флаги для открытия файлов
#define VTPC_O_RDONLY     0x0001
#define VTPC_O_WRONLY     0x0002  
#define VTPC_O_RDWR       0x0004
#define VTPC_O_CREAT      0x0040
#define VTPC_O_DIRECT     0x0010  // Прямой доступ к диску
#define VTPC_O_SYNC       0x0020  // Синхронная запись

// Основные функции API

int vtpc_open(const char *path, int flags);
int vtpc_close(int fd);
ssize_t vtpc_read(int fd, void *buf, size_t count);
ssize_t vtpc_write(int fd, const void *buf, size_t count);
off_t vtpc_lseek(int fd, off_t offset, int whence);
int vtpc_fsync(int fd);
void vtpc_cleanup(void);

// Функции для работы с прямым доступом
void* vtpc_alloc_aligned_buffer(size_t size);
void vtpc_free_aligned_buffer(void *buffer);
ssize_t vtpc_read_aligned(int fd, void *aligned_buf, size_t count);
ssize_t vtpc_write_aligned(int fd, const void *aligned_buf, size_t count);
ssize_t vtpc_read_direct(int fd, void *aligned_buf, size_t count);
ssize_t vtpc_write_direct(int fd, const void *aligned_buf, size_t count);

#endif // VTPC_H