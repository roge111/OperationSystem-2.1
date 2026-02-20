/**
 * @file vtpc.c
 * @brief Реализация виртуальной файловой системы с прямым доступом к диску
 *
 * Этот файл содержит реализацию виртуальной файловой системы, которая
 * предоставляет функции для работы с файлами с возможностью прямого
 * доступа к диску (обход кэша операционной системы).
 *
 * Основные возможности:
 * - Открытие/закрытие файлов
 * - Чтение/запись данных
 * - Позиционирование в файле
 * - Синхронизация данных с диском
 * - Прямой доступ к диску (обход кэша ОС)
 * - Кэширование блоков данных
 */

// vtpc_raw.c - полный код БЕЗ libc оберток
#include "vtpc.h"
#include <linux/fcntl.h>
#include <sys/syscall.h>
#include <stdatomic.h>
#include <errno.h>
#include <sys/mman.h>
#include <stdint.h>
#include <time.h>
#include <stdio.h>

#define VTPC_O_RDONLY 0x0001
#define VTPC_O_WRONLY 0x0002
#define VTPC_O_RDWR 0x0004
#define VTPC_O_DIRECT 0x0010 // Прямой доступ к диску (O_DIRECT)
#define VTPC_O_SYNC 0x0020   // Синхронная запись

// Определяем O_DIRECT для прямого доступа к диску
#ifndef O_DIRECT
#define O_DIRECT 00040000 /* прямой доступ к диску */
#endif

// Опредеояем макросы для mmap
#ifndef PROT_READ
#define PROT_READ 0x1
#endif

#ifndef PROT_WRITE
#define PROT_WRITE 0x2
#endif

#ifndef MAP_PRIVATE
#define MAP_PRIVATE 0x02
#endif

#ifndef MAP_ANONYMOUS
#define MAP_ANONYMOUS 0x20
#endif

#ifndef MAP_POPULATE
#define MAP_POPULATE 0x8000
#endif

// Определяем макросы для lseek
#ifndef SEEK_SET
#define SEEK_SET 0
#endif

#ifndef SEEK_CUR
#define SEEK_CUR 1
#endif

#ifndef SEEK_END
#define SEEK_END 2
#endif

#ifndef CLOCK_REALTIME
#define CLOCK_REALTIME 0
#endif

// Номера системных вызовов для x86_64
#ifndef SYS_open
#define SYS_open 2
#endif

#ifndef SYS_close
#define SYS_close 3
#endif

#ifndef SYS_read
#define SYS_read 0
#endif

#ifndef SYS_write
#define SYS_write 1
#endif

#ifndef SYS_lseek
#define SYS_lseek 8
#endif

#ifndef SYS_fsync
#define SYS_fsync 74
#endif

#ifndef SYS_mmap
#define SYS_mmap 9
#endif

#ifndef SYS_munmap
#define SYS_munmap 11
#endif

#ifndef SYS_clock_gettime
#define SYS_clock_gettime 228
#endif

#ifndef SYS_fallocate
#define SYS_fallocate 285
#endif

// Размеры
#define BLOCK_SIZE 4096
#define DIRECT_ALIGNMENT 512 // Выравнивание для прямого доступа
#define MAX_CACHE_BLOCKS 1024

// Глобальные переменные для статистики кэша
int vtpc_cache_hits = 0;
int vtpc_cache_misses = 0;
int vtpc_cache_total = 0;

// ========== ПРОТОТИПЫ ФУНКЦИЙ ==========

ssize_t vtpc_read_aligned(int fd, void *aligned_buf, size_t count);
ssize_t vtpc_write_aligned(int fd, const void *aligned_buf, size_t count);

// ========== СИСТЕМНЫЕ ВЫЗОВЫ НАПРЯМУЮ ==========

/**
 * @brief Системный вызов ядра Linux
 *
 * Выполняет системный вызов с указанным номером и аргументами.
 *
 * @param number Номер системного вызова
 * @param ... Аргументы системного вызова (зависят от конкретного вызова)
 * @return Результат системного вызова (зависит от конкретного вызова)
 */
long syscall(long number, ...);

/**
 * @brief Открытие файла через системный вызов
 *
 * Открывает файл по указанному пути с заданными флагами и правами доступа.
 *
 * @param path Путь к файлу
 * @param flags Флаги открытия файла (O_RDONLY, O_WRONLY, O_RDWR и др.)
 * @param mode Права доступа к файлу (если создается новый)
 * @return Дескриптор файла или -1 в случае ошибки
 */
static inline int raw_open(const char *path, int flags, int mode)
{
    return syscall(SYS_open, path, flags, mode);
}

/**
 * @brief Закрытие файла через системный вызов
 *
 * Закрывает файл по указанному дескриптору.
 *
 * @param fd Дескриптор файла
 * @return 0 в случае успеха, -1 в случае ошибки
 */
static inline int raw_close(int fd)
{
    return syscall(SYS_close, fd);
}

/**
 * @brief Чтение данных из файла через системный вызов
 *
 * Читает указанное количество байт из файла в буфер.
 *
 * @param fd Дескриптор файла
 * @param buf Буфер для чтения данных
 * @param count Количество байт для чтения
 * @return Количество прочитанных байт или -1 в случае ошибки
 */
static inline ssize_t raw_read(int fd, void *buf, size_t count)
{
    return syscall(SYS_read, fd, buf, count);
}

/**
 * @brief Запись данных в файл через системный вызов
 *
 * Записывает указанное количество байт из буфера в файл.
 *
 * @param fd Дескриптор файла
 * @param buf Буфер с данными для записи
 * @param count Количество байт для записи
 * @return Количество записанных байт или -1 в случае ошибки
 */
static inline ssize_t raw_write(int fd, const void *buf, size_t count)
{
    return syscall(SYS_write, fd, buf, count);
}

/**
 * @brief Позиционирование в файле через системный вызов
 *
 * Устанавливает позицию в файле согласно указанному смещению и режиму.
 *
 * @param fd Дескриптор файла
 * @param offset Смещение относительно точки отсчета
 * @param whence Точка отсчета (SEEK_SET, SEEK_CUR, SEEK_END)
 * @return Новая позиция в файле или -1 в случае ошибки
 */
static inline off_t raw_lseek(int fd, off_t offset, int whence)
{
    return syscall(SYS_lseek, fd, offset, whence);
}

/**
 * @brief Синхронизация данных файла с диском через системный вызов
 *
 * Принудительно записывает все буферизованные данные файла на диск.
 *
 * @param fd Дескриптор файла
 * @return 0 в случае успеха, -1 в случае ошибки
 */
static inline int raw_fsync(int fd)
{
    return syscall(SYS_fsync, fd);
}

/**
 * @brief Выделение выровненной памяти через системный вызов mmap
 *
 * Выделяет блок памяти указанного размера с выравниванием по границе DIRECT_ALIGNMENT байт.
 *
 * @param size Размер выделяемой памяти
 * @return Указатель на выделенную выровненную память или NULL в случае ошибки
 */
static inline void *raw_mmap_aligned(size_t size)
{
    // Создаем выровненную память для прямого доступа
    size_t total_size = size + DIRECT_ALIGNMENT - 1;
    void *ptr = (void *)syscall(SYS_mmap,
                                NULL, total_size,
                                PROT_READ | PROT_WRITE,
                                MAP_PRIVATE | MAP_ANONYMOUS,
                                -1, 0);

    if ((long)ptr < 0 && (long)ptr > -4096)
    {
        return NULL;
    }

    // Выравниваем указатель по границе 512 байт
    uintptr_t addr = (uintptr_t)ptr;
    uintptr_t aligned_addr = (addr + DIRECT_ALIGNMENT - 1) & ~(DIRECT_ALIGNMENT - 1);

    return (void *)aligned_addr;
}

/**
 * @brief Освобождение памяти через системный вызов munmap
 *
 * Освобождает блок памяти, выделенный через mmap.
 *
 * @param addr Указатель на освобождаемую память
 * @param size Размер освобождаемой памяти
 * @return 0 в случае успеха, -1 в случае ошибки
 */
static inline int raw_munmap(void *addr, size_t size)
{
    // Для выровненной памяти нужно освобождать исходный указатель
    uintptr_t orig_addr = (uintptr_t)addr;
    if (orig_addr & (DIRECT_ALIGNMENT - 1))
    {
        orig_addr = orig_addr & ~(DIRECT_ALIGNMENT - 1);
    }
    size_t total_size = size + DIRECT_ALIGNMENT - 1;
    return syscall(SYS_munmap, (void *)orig_addr, total_size);
}

/**
 * @brief Получение текущего времени через системный вызов
 *
 * Получает текущее время системы в секундах.
 *
 * @return Текущее время в секундах
 */
static inline time_t raw_time(void)
{
    struct timespec ts;
    syscall(SYS_clock_gettime, CLOCK_REALTIME, &ts);
    return ts.tv_sec;
}

/**
 * @brief Предварительное выделение места под файл через системный вызов
 *
 * Предварительно выделяет место на диске под файл.
 *
 * @param fd Дескриптор файла
 * @param mode Режим выделения
 * @param offset Смещение от начала файла
 * @param len Длина выделяемого пространства
 * @return 0 в случае успеха, -1 в случае ошибки
 */
static inline int raw_fallocate(int fd, int mode, off_t offset, off_t len)
{
    return syscall(SYS_fallocate, fd, mode, offset, len);
}

// ========== ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ==========

/**
 * @brief Заполнение памяти заданным значением
 *
 * Заполняет блок памяти указанным значением.
 *
 * @param ptr Указатель на блок памяти
 * @param value Значение для заполнения
 * @param num Количество байт для заполнения
 */
static void raw_memset(void *ptr, int value, size_t num)
{
    unsigned char *p = (unsigned char *)ptr;
    for (size_t i = 0; i < num; i++)
    {
        p[i] = (unsigned char)value;
    }
}

/**
 * @brief Копирование памяти
 *
 * Копирует блок памяти из одного места в другое.
 *
 * @param dest Указатель на место назначения
 * @param src Указатель на источник данных
 * @param n Количество байт для копирования
 */
static void raw_memcpy(void *dest, const void *src, size_t n)
{
    unsigned char *d = (unsigned char *)dest;
    const unsigned char *s = (const unsigned char *)src;
    for (size_t i = 0; i < n; i++)
    {
        d[i] = s[i];
    }
}

/**
 * @brief Проверка выравнивания указателя
 *
 * Проверяет, выровнен ли указатель по заданной границе.
 *
 * @param ptr Проверяемый указатель
 * @param alignment Граница выравнивания
 * @return 1 если указатель выровнен, 0 если нет
 */
static inline int is_aligned(void *ptr, size_t alignment)
{
    uintptr_t addr = (uintptr_t)ptr;
    return (addr & (alignment - 1)) == 0;
}

// ========== СТРУКТУРЫ ДАННЫХ ==========

/**
 * @brief Структура кэш-блока для размеченного массива
 *
 * Представляет блок данных в кэше файловой системы.
 * Все данные блока и LRU информация хранятся в одной структуре.
 */
typedef struct CacheBlock
{
    // Данные блока
    int fd;                  // Дескриптор файла 
    off_t block_number;      // Номер блока
    char *data;              // Выровненная память для прямого доступа 
    int dirty;               // Флаг "грязного" блока (требует записи на диск)
    time_t last_access;      // Время последнего доступа
    void *original_ptr;      // Исходный указатель для освобождения памяти
    
    // LRU информация (встроена в ту же структуру)
    int lru_prev;            // Индекс предыдущего блока в LRU списке (-1 если нет)
    int lru_next;            // Индекс следующего блока в LRU списке (-1 если нет)
    int valid;               // Флаг валидности блока (1 - используется, 0 - свободен)
} CacheBlock;

/**
 * @brief Структура информации о файле
 *
 * Содержит информацию о состоянии открытого файла.
 */
typedef struct FileInfo
{
    int fd;            // Дескриптор файла 
    off_t file_size;   // Размер файла
    off_t position;    // Текущая позиция в файле
    int use_direct_io; // Флаг использования прямого доступа
} FileInfo;

/**
 * @brief Структура буфера прямого доступа
 *
 * Представляет буфер памяти, выровненный для прямого доступа к диску.
 */
typedef struct DirectIOBuffer
{
    char *buffer;       // Выровненный буфер 
    void *original_ptr; // Исходный указатель для освобождения
    size_t size;        // Размер буфера
} DirectIOBuffer;

/**
 * @brief Структура менеджера кэша
 *
 * Управляет размеченным массивом кэш-блоков.
 */
typedef struct CacheManager
{
    CacheBlock *blocks;       // Размеченный массив кэш-блоков
    int capacity;             // Максимальное количество блоков
    int block_count;          // Текущее количество блоков в кэше
    int lru_head;             // Индекс самого свежего блока (-1 если нет)
    int lru_tail;             // Индекс самого старого блока (-1 если нет)
    volatile int lock;        // Спинлок для синхронизации доступа к кэшу
} CacheManager;

static FileInfo *open_files[1024];
static CacheManager cache_manager;

// ========== ИНИЦИАЛИЗАЦИЯ КЭША ==========

/**
 * @brief Инициализация менеджера кэша
 *
 * Выделяет память под размеченный массив и инициализирует структуру кэша.
 *
 * @param capacity Максимальное количество блоков в кэше
 * @return 0 в случае успеха, -1 в случае ошибки
 */
static int init_cache_manager(int capacity)
{
    // Выделяем память под размеченный массив структур
    cache_manager.blocks = (CacheBlock*)raw_mmap_aligned(sizeof(CacheBlock) * capacity);
    if (!cache_manager.blocks)
        return -1;
    
    // Инициализация массива
    for (int i = 0; i < capacity; i++) {
        cache_manager.blocks[i].fd = -1;
        cache_manager.blocks[i].block_number = -1;
        cache_manager.blocks[i].data = NULL;
        cache_manager.blocks[i].dirty = 0;
        cache_manager.blocks[i].last_access = 0;
        cache_manager.blocks[i].original_ptr = NULL;
        cache_manager.blocks[i].lru_prev = -1;
        cache_manager.blocks[i].lru_next = -1;
        cache_manager.blocks[i].valid = 0;
    }
    
    cache_manager.capacity = capacity;
    cache_manager.block_count = 0;
    cache_manager.lru_head = -1;
    cache_manager.lru_tail = -1;
    cache_manager.lock = 0;
    
    return 0;
}

// ========== СПИНЛОКИ ==========

/**
 * @brief Захват спинлока
 *
 * Выполняет захват спинлока с активным ожиданием.
 *
 * @param lock Указатель на спинлок
 */
static inline void spin_lock(volatile int *lock)
{
    while (__sync_lock_test_and_set(lock, 1))
    {
#ifdef __x86_64__
        __asm__ __volatile__("pause" ::: "memory");
#endif
    }
}

/**
 * @brief Освобождение спинлока
 *
 * Освобождает захваченный спинлок.
 *
 * @param lock Указатель на спинлок
 */
static inline void spin_unlock(volatile int *lock)
{
    __sync_lock_release(lock);
}

// ========== ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ КЭША ==========

/**
 * @brief Получение номера блока по позиции в файле
 *
 * Вычисляет номер блока на основе позиции в файле.
 *
 * @param position Позиция в файле
 * @return Номер блока
 */
static off_t get_block_number(off_t position)
{
    return position / BLOCK_SIZE;
}

/**
 * @brief Получение смещения внутри блока по позиции в файле
 *
 * Вычисляет смещение внутри блока на основе позиции в файле.
 *
 * @param position Позиция в файле
 * @return Смещение внутри блока
 */
static off_t get_block_offset(off_t position)
{
    return position % BLOCK_SIZE;
}

// ========== ФУНКЦИИ РАБОТЫ С LRU СПИСКОМ ==========

/**
 * @brief Переместить блок в начало LRU списка
 *
 * @param idx Индекс блока в массиве
 */
static void move_to_lru_head(int idx)
{
    if (idx < 0 || idx >= cache_manager.capacity || !cache_manager.blocks[idx].valid)
        return;
    
    // Если блок уже в начале, ничего не делаем
    if (cache_manager.lru_head == idx)
        return;
    
    // Удаляем блок из текущей позиции в списке
    int prev = cache_manager.blocks[idx].lru_prev;
    int next = cache_manager.blocks[idx].lru_next;
    
    if (prev != -1)
        cache_manager.blocks[prev].lru_next = next;
    if (next != -1)
        cache_manager.blocks[next].lru_prev = prev;
    
    // Если блок был хвостом, обновляем хвост
    if (cache_manager.lru_tail == idx)
        cache_manager.lru_tail = prev;
    
    // Вставляем блок в начало
    cache_manager.blocks[idx].lru_prev = -1;
    cache_manager.blocks[idx].lru_next = cache_manager.lru_head;
    
    if (cache_manager.lru_head != -1)
        cache_manager.blocks[cache_manager.lru_head].lru_prev = idx;
    
    cache_manager.lru_head = idx;
    
    // Если список был пуст, обновляем хвост
    if (cache_manager.lru_tail == -1)
        cache_manager.lru_tail = idx;
}

/**
 * @brief Добавить новый блок в начало LRU списка
 *
 * @param idx Индекс нового блока
 */
static void add_to_lru_head(int idx)
{
    if (idx < 0 || idx >= cache_manager.capacity)
        return;
    
    cache_manager.blocks[idx].lru_prev = -1;
    cache_manager.blocks[idx].lru_next = cache_manager.lru_head;
    
    if (cache_manager.lru_head != -1)
        cache_manager.blocks[cache_manager.lru_head].lru_prev = idx;
    
    cache_manager.lru_head = idx;
    
    if (cache_manager.lru_tail == -1)
        cache_manager.lru_tail = idx;
    
    cache_manager.block_count++;
}

/**
 * @brief Удалить блок из LRU списка
 *
 * @param idx Индекс удаляемого блока
 */
static void remove_from_lru(int idx)
{
    if (idx < 0 || idx >= cache_manager.capacity || !cache_manager.blocks[idx].valid)
        return;
    
    int prev = cache_manager.blocks[idx].lru_prev;
    int next = cache_manager.blocks[idx].lru_next;
    
    if (prev != -1)
        cache_manager.blocks[prev].lru_next = next;
    if (next != -1)
        cache_manager.blocks[next].lru_prev = prev;
    
    if (cache_manager.lru_head == idx)
        cache_manager.lru_head = next;
    if (cache_manager.lru_tail == idx)
        cache_manager.lru_tail = prev;
    
    cache_manager.blocks[idx].lru_prev = -1;
    cache_manager.blocks[idx].lru_next = -1;
    cache_manager.block_count--;
}

/**
 * @brief Получить индекс наименее используемого блока
 *
 * @return Индекс блока для вытеснения или -1 если кэш пуст
 */
static int get_lru_victim_index(void)
{
    return cache_manager.lru_tail;
}

// ========== ФУНКЦИИ ПРЯМОГО ДОСТУПА К ДИСКУ ==========

/**
 * @brief Создание буфера прямого доступа
 *
 * Создает буфер памяти, выровненный для прямого доступа к диску.
 *
 * @param size Размер буфера
 * @return Структура DirectIOBuffer с указателями на буфер и исходную память
 */
static DirectIOBuffer create_direct_io_buffer(size_t size)
{
    DirectIOBuffer buffer = {0};

    // Округляем размер до выровненного значения
    size_t aligned_size = ((size + DIRECT_ALIGNMENT - 1) / DIRECT_ALIGNMENT) * DIRECT_ALIGNMENT;

    // Выделяем память с запасом для выравнивания
    size_t total_size = aligned_size + DIRECT_ALIGNMENT - 1;
    void *ptr = (void *)syscall(SYS_mmap,
                                NULL, total_size,
                                PROT_READ | PROT_WRITE,
                                MAP_PRIVATE | MAP_ANONYMOUS,
                                -1, 0);

    if ((long)ptr < 0 && (long)ptr > -4096)
    {
        return buffer;
    }

    // Выравниваем указатель
    uintptr_t addr = (uintptr_t)ptr;
    uintptr_t aligned_addr = (addr + DIRECT_ALIGNMENT - 1) & ~(DIRECT_ALIGNMENT - 1);

    buffer.buffer = (char *)aligned_addr;
    buffer.original_ptr = ptr;
    buffer.size = aligned_size;

    return buffer;
}

/**
 * @brief Освобождение буфера прямого доступа
 *
 * Освобождает память, выделенную для буфера прямого доступа.
 *
 * @param buffer Указатель на структуру DirectIOBuffer
 */
static void free_direct_io_buffer(DirectIOBuffer *buffer)
{
    if (buffer->original_ptr)
    {
        size_t total_size = buffer->size + DIRECT_ALIGNMENT - 1;
        syscall(SYS_munmap, buffer->original_ptr, total_size);
        buffer->buffer = NULL;
        buffer->original_ptr = NULL;
    }
}

/**
 * @brief Прямое чтение с диска в обход кэша ОС
 *
 * Выполняет чтение данных из файла напрямую с диска, обходя кэш операционной системы.
 *
 * @param fd Дескриптор файла
 * @param buf Буфер для чтения данных
 * @param count Количество байт для чтения
 * @param offset Смещение в файле
 * @return Количество прочитанных байт или -1 в случае ошибки
 */
static ssize_t direct_disk_read(int fd, void *buf, size_t count, off_t offset)
{
    // Сохраняем текущую позицию
    off_t current_pos = raw_lseek(fd, 0, SEEK_CUR);
    if (current_pos < 0)
        return -1;

    // Перемещаемся к нужной позиции
    if (raw_lseek(fd, offset, SEEK_SET) < 0)
    {
        raw_lseek(fd, current_pos, SEEK_SET);
        return -1;
    }

    // Читаем данные напрямую с диска
    ssize_t bytes_read = raw_read(fd, buf, count);

    // Восстанавливаем позицию
    raw_lseek(fd, current_pos, SEEK_SET);

    return bytes_read;
}

/**
 * @brief Прямая запись на диск в обход кэша ОС
 *
 * Выполняет запись данных в файл напрямую на диск, обходя кэш операционной системы.
 *
 * @param fd Дескриптор файла
 * @param buf Буфер с данными для записи
 * @param count Количество байт для записи
 * @param offset Смещение в файле
 * @return Количество записанных байт или -1 в случае ошибки
 */
static ssize_t direct_disk_write(int fd, const void *buf, size_t count, off_t offset)
{
    // Сохраняем текущую позицию
    off_t current_pos = raw_lseek(fd, 0, SEEK_CUR);
    if (current_pos < 0)
        return -1;

    // Перемещаемся к нужной позиции
    if (raw_lseek(fd, offset, SEEK_SET) < 0)
    {
        raw_lseek(fd, current_pos, SEEK_SET);
        return -1;
    }

    // Записываем данные напрямую на диск
    ssize_t bytes_written = raw_write(fd, buf, count);

    // Восстанавливаем позицию
    raw_lseek(fd, current_pos, SEEK_SET);

    return bytes_written;
}

// ========== ОСНОВНЫЕ ФУНКЦИИ КЭША ==========

/**
 * @brief Чтение блока данных с диска через прямой доступ
 *
 * Выполняет чтение блока данных с диска в кэш-блок через прямой доступ.
 *
 * @param block Указатель на кэш-блок для чтения данных
 */
static void read_block_from_disk_direct(CacheBlock *block)
{
    FileInfo *file_info = open_files[block->fd];
    if (!file_info)
        return;

    off_t pos = block->block_number * BLOCK_SIZE;

    // Используем прямое чтение с диска
    ssize_t bytes = direct_disk_read(file_info->fd, block->data, BLOCK_SIZE, pos);

    if (bytes < BLOCK_SIZE)
    {
        if (bytes > 0)
        {
            raw_memset(block->data + bytes, 0, BLOCK_SIZE - bytes);
        }
        else
        {
            raw_memset(block->data, 0, BLOCK_SIZE);
        }
    }

    block->dirty = 0;
    block->last_access = raw_time();
}

/**
 * @brief Запись блока данных на диск через прямой доступ
 *
 * Выполняет запись "грязного" кэш-блока на диск через прямой доступ.
 *
 * @param block Указатель на кэш-блок для записи
 */
static void write_block_to_disk_direct(CacheBlock *block)
{
    FileInfo *file_info = open_files[block->fd];
    if (!file_info || !block->dirty)
        return;

    off_t pos = block->block_number * BLOCK_SIZE;

    // Используем прямую запись на диск
    ssize_t written = direct_disk_write(file_info->fd, block->data, BLOCK_SIZE, pos);

    if (written == BLOCK_SIZE)
    {
        block->dirty = 0;
    }
}

/**
 * @brief Поиск блока в размеченном массиве
 *
 * Ищет блок с указанными fd и block_number в кэше.
 * Если блок найден, обновляет LRU порядок.
 *
 * @param fd Дескриптор файла
 * @param block_number Номер блока
 * @return Указатель на найденный блок или NULL, если блок не найден
 */
static CacheBlock *find_block_in_cache(int fd, off_t block_number)
{
    vtpc_cache_total++;
    
    // Линейный поиск по всему массиву (так как у нас нет хеш-таблицы)
    for (int i = 0; i < cache_manager.capacity; i++) {
        if (cache_manager.blocks[i].valid && 
            cache_manager.blocks[i].fd == fd && 
            cache_manager.blocks[i].block_number == block_number) {
            
            // Попадание в кэш
            cache_manager.blocks[i].last_access = raw_time();
            vtpc_cache_hits++;
            
            // Перемещаем в начало LRU списка
            move_to_lru_head(i);
            
            return &cache_manager.blocks[i];
        }
    }
    
    vtpc_cache_misses++;
    return NULL;
}

/**
 * @brief Выделение нового блока в размеченном массиве
 *
 * Выделяет новый блок в кэше для указанного файла и номера блока.
 * Если кэш переполнен, вытесняет LRU блок.
 *
 * @param fd Дескриптор файла
 * @param block_number Номер блока
 * @return Указатель на новый блок или NULL в случае ошибки
 */
static CacheBlock *allocate_new_block(int fd, off_t block_number)
{
    int free_idx = -1;
    
    // Сначала ищем свободный слот
    for (int i = 0; i < cache_manager.capacity; i++) {
        if (!cache_manager.blocks[i].valid) {
            free_idx = i;
            break;
        }
    }
    
    // Если нет свободных слотов, вытесняем LRU блок
    if (free_idx == -1) {
        int victim_idx = get_lru_victim_index();
        if (victim_idx < 0)
            return NULL;
        
        // Освобождаем вытесняемый блок
        CacheBlock *victim = &cache_manager.blocks[victim_idx];
        
        // Если блок "грязный", записываем на диск
        if (victim->dirty) {
            FileInfo *file_info = open_files[victim->fd];
            if (file_info) {
                if (file_info->use_direct_io) {
                    write_block_to_disk_direct(victim);
                } else {
                    off_t pos = victim->block_number * BLOCK_SIZE;
                    off_t old_pos = raw_lseek(file_info->fd, 0, SEEK_CUR);
                    if (old_pos >= 0 && raw_lseek(file_info->fd, pos, SEEK_SET) >= 0) {
                        raw_write(file_info->fd, victim->data, BLOCK_SIZE);
                        raw_lseek(file_info->fd, old_pos, SEEK_SET);
                    }
                }
            }
        }
        
        // Освобождаем память данных
        if (victim->original_ptr) {
            raw_munmap(victim->original_ptr, BLOCK_SIZE + DIRECT_ALIGNMENT - 1);
            victim->data = NULL;
            victim->original_ptr = NULL;
        }
        
        // Удаляем из LRU списка
        remove_from_lru(victim_idx);
        
        // Помечаем как невалидный
        victim->valid = 0;
        free_idx = victim_idx;
    }
    
    if (free_idx < 0)
        return NULL;
    
    CacheBlock *new_block = &cache_manager.blocks[free_idx];
    
    // Инициализация блока
    raw_memset(new_block, 0, sizeof(CacheBlock));
    new_block->fd = fd;
    new_block->block_number = block_number;
    new_block->dirty = 0;
    new_block->last_access = raw_time();
    new_block->valid = 1;
    
    // Выделение выровненной памяти для данных
    DirectIOBuffer buffer = create_direct_io_buffer(BLOCK_SIZE);
    if (!buffer.buffer) {
        new_block->valid = 0;
        return NULL;
    }
    
    new_block->data = buffer.buffer;
    new_block->original_ptr = buffer.original_ptr;
    
    // Проверяем информацию о файле
    FileInfo *file_info = open_files[fd];
    if (!file_info) {
        raw_munmap(buffer.original_ptr, buffer.size + DIRECT_ALIGNMENT - 1);
        new_block->valid = 0;
        return NULL;
    }
    
    // Чтение данных с диска
    off_t pos = block_number * BLOCK_SIZE;
    ssize_t bytes_read = direct_disk_read(file_info->fd,
                                         new_block->data,
                                         BLOCK_SIZE,
                                         pos);
    
    if (bytes_read < BLOCK_SIZE) {
        if (bytes_read > 0) {
            raw_memset(new_block->data + bytes_read, 0, BLOCK_SIZE - bytes_read);
        } else if (bytes_read < 0) {
            raw_munmap(new_block->original_ptr, BLOCK_SIZE + DIRECT_ALIGNMENT - 1);
            new_block->valid = 0;
            return NULL;
        } else {
            raw_memset(new_block->data, 0, BLOCK_SIZE);
        }
    }
    
    // Добавляем в начало LRU списка
    add_to_lru_head(free_idx);
    
    return new_block;
}

// ========== API ФУНКЦИИ ==========

/**
 * @brief Открытие файла с флагами и правами доступа
 *
 * Открывает файл по указанному пути с заданными флагами и правами доступа.
 *
 * @param path Путь к файлу
 * @param flags Флаги открытия файла (системные флаги O_*)
 * @param mode Права доступа к файлу (используются при создании нового файла)
 * @return Виртуальный дескриптор файла или -1 в случае ошибки
 */
int vtpc_open(const char *path, int flags, int mode)
{
    static int initialized = 0;
    if (!initialized)
    {
        // Инициализируем менеджер кэша
        if (init_cache_manager(MAX_CACHE_BLOCKS) < 0) {
            return -1;
        }
        raw_memset(open_files, 0, sizeof(open_files));
        initialized = 1;
    }

    int sys_flags = 0;
    int use_direct_io = 0;

    // Копируем флаги как есть
    sys_flags = flags;

    // Проверяем, передан ли O_DIRECT
    if (flags & 00040000) { // O_DIRECT
        use_direct_io = 1;
    } else {
        use_direct_io = 0;
        sys_flags &= ~O_DIRECT; // Убеждаемся, что O_DIRECT не установлен
    }

    int actual_mode = mode;
    if (actual_mode == 0)
    {
        actual_mode = 0666;
    }

    // Открываем файл
    int sys_fd = raw_open(path, sys_flags, actual_mode);

    // Если открываем с O_DIRECT и не получилось, пробуем без него
    if (sys_fd < 0 && (sys_flags & O_DIRECT))
    {
        sys_flags &= ~O_DIRECT;
        sys_fd = raw_open(path, sys_flags, actual_mode);
        if (sys_fd < 0)
            return -1;
        use_direct_io = 0;
    }
    else if (sys_fd < 0)
    {
        return -1;
    }

    // Получаем размер файла
    off_t size = raw_lseek(sys_fd, 0, SEEK_END);
    if (size < 0 || raw_lseek(sys_fd, 0, SEEK_SET) < 0)
    {
        raw_close(sys_fd);
        return -1;
    }

    // Ищем свободный слот
    int vtpc_fd = -1;
    for (int i = 0; i < 1024; i++)
    {
        if (!open_files[i])
        {
            vtpc_fd = i;
            break;
        }
    }

    if (vtpc_fd < 0)
    {
        raw_close(sys_fd);
        return -1;
    }

    FileInfo *info = raw_mmap_aligned(sizeof(FileInfo));
    if (!info)
    {
        raw_close(sys_fd);
        return -1;
    }

    info->fd = sys_fd;
    info->file_size = size;
    info->position = 0;
    info->use_direct_io = use_direct_io;

    open_files[vtpc_fd] = info;
    return vtpc_fd;
}

/**
 * @brief Закрытие файла
 *
 * Закрывает файл по указанному виртуальному дескриптору.
 * Записывает все "грязные" блоки на диск и освобождает ресурсы.
 *
 * @param fd Виртуальный дескриптор файла
 * @return 0 в случае успеха, -1 в случае ошибки
 */
int vtpc_close(int fd)
{
    if (fd < 0 || fd >= 1024 || !open_files[fd])
        return -1;

    spin_lock(&cache_manager.lock);

    // Пишем dirty блоки для этого файла
    for (int i = 0; i < cache_manager.capacity; i++) {
        CacheBlock *block = &cache_manager.blocks[i];
        if (block->valid && block->fd == fd && block->dirty)
        {
            FileInfo *file_info = open_files[fd];
            if (file_info)
            {
                if (file_info->use_direct_io)
                {
                    write_block_to_disk_direct(block);
                }
                else
                {
                    off_t pos = block->block_number * BLOCK_SIZE;
                    off_t old_pos = raw_lseek(file_info->fd, 0, SEEK_CUR);
                    if (old_pos >= 0 && raw_lseek(file_info->fd, pos, SEEK_SET) >= 0)
                    {
                        raw_write(file_info->fd, block->data, BLOCK_SIZE);
                        raw_lseek(file_info->fd, old_pos, SEEK_SET);
                    }
                }
            }
        }
    }

    // Удаляем блоки файла из кэша
    for (int i = 0; i < cache_manager.capacity; i++) {
        CacheBlock *block = &cache_manager.blocks[i];
        if (block->valid && block->fd == fd)
        {
            // Удаляем из LRU
            remove_from_lru(i);
            
            // Освобождаем память
            if (block->original_ptr)
            {
                raw_munmap(block->original_ptr, BLOCK_SIZE + DIRECT_ALIGNMENT - 1);
                block->data = NULL;
                block->original_ptr = NULL;
            }
            
            // Помечаем как невалидный
            block->valid = 0;
        }
    }

    spin_unlock(&cache_manager.lock);

    // Закрываем файл и освобождаем память
    FileInfo *info = open_files[fd];
    raw_close(info->fd);
    raw_munmap(info, sizeof(FileInfo));
    open_files[fd] = 0;

    return 0;
}

/**
 * @brief Чтение данных из файла с выровненным буфером
 *
 * Читает указанное количество байт из файла в выровненный буфер.
 * Используется для прямого доступа к диску.
 *
 * @param fd Виртуальный дескриптор файла
 * @param aligned_buf Выровненный буфер для чтения данных
 * @param count Количество байт для чтения
 * @return Количество прочитанных байт или -1 в случае ошибки
 */
ssize_t vtpc_read_aligned(int fd, void *aligned_buf, size_t count)
{
    if (fd < 0 || fd >= 1024 || !open_files[fd] || !aligned_buf)
        return -1;

    if (!is_aligned(aligned_buf, DIRECT_ALIGNMENT))
        return -1;

    FileInfo *info = open_files[fd];

    if (info->position >= info->file_size)
        return 0;

    size_t to_read = count;
    if (info->position + (off_t)to_read > info->file_size)
    {
        to_read = info->file_size - info->position;
    }

    size_t aligned_count = ((to_read + DIRECT_ALIGNMENT - 1) / DIRECT_ALIGNMENT) * DIRECT_ALIGNMENT;

    ssize_t bytes_read = direct_disk_read(info->fd, aligned_buf, aligned_count, info->position);

    if (bytes_read > 0)
    {
        if ((size_t)bytes_read > to_read)
        {
            bytes_read = to_read;
        }
        info->position += bytes_read;
    }

    return bytes_read;
}

/**
 * @brief Чтение данных из файла
 *
 * Читает указанное количество байт из файла в буфер.
 *
 * @param fd Виртуальный дескриптор файла
 * @param buf Буфер для чтения данных
 * @param count Количество байт для чтения
 * @return Количество прочитанных байт или -1 в случае ошибки
 */
ssize_t vtpc_read(int fd, void *buf, size_t count)
{
    if (fd < 0 || fd >= 1024 || !open_files[fd] || !buf)
        return -1;

    FileInfo *info = open_files[fd];
    
    if (info->position >= info->file_size)
        return 0;

    size_t to_read = count;
    if (info->position + (off_t)to_read > info->file_size)
    {
        to_read = info->file_size - info->position;
    }

    if (info->use_direct_io)
    {
        // Для O_DIRECT нужна специальная обработка
        if (!is_aligned(buf, DIRECT_ALIGNMENT) || 
            (to_read % DIRECT_ALIGNMENT) != 0 ||
            (info->position % DIRECT_ALIGNMENT) != 0)
        {
            // Используем временный выровненный буфер
            char *aligned_buf = vtpc_alloc_aligned_buffer(to_read);
            if (!aligned_buf)
                return -1;
            
            ssize_t result = vtpc_read_aligned(fd, aligned_buf, to_read);
            if (result > 0) {
                raw_memcpy(buf, aligned_buf, result);
            }
            
            vtpc_free_aligned_buffer(aligned_buf);
            return result;
        }
        
        return vtpc_read_aligned(fd, buf, to_read);
    }
    
    // Обычное чтение без O_DIRECT с использованием кэша
    size_t total = 0;
    char *buffer = (char*)buf;

    while (total < to_read) {
        off_t pos = info->position + (off_t)total;
        off_t block_num = get_block_number(pos);
        off_t offset = get_block_offset(pos);
        
        size_t in_block = BLOCK_SIZE - offset;
        size_t needed = to_read - total;
        size_t copy = (in_block < needed) ? in_block : needed;
        
        spin_lock(&cache_manager.lock);
        
        CacheBlock *block = find_block_in_cache(fd, block_num);
        
        if (!block) {
            block = allocate_new_block(fd, block_num);
            if (!block) {
                spin_unlock(&cache_manager.lock);
                info->position += (off_t)total;
                return total;
            }
        }
        
        raw_memcpy(buffer + total, block->data + offset, copy);
        total += copy;
        
        spin_unlock(&cache_manager.lock);
    }

    info->position += (off_t)total;
    return total;
}

/**
 * @brief Запись данных в файл
 *
 * Записывает указанное количество байт из буфера в файл.
 *
 * @param fd Виртуальный дескриптор файла
 * @param buf Буфер с данными для записи
 * @param count Количество байт для записи
 * @return Количество записанных байт или -1 в случае ошибки
 */
ssize_t vtpc_write(int fd, const void *buf, size_t count)
{
    if (fd < 0 || fd >= 1024 || !open_files[fd] || !buf)
        return -1;

    FileInfo *info = open_files[fd];

    if (info->use_direct_io)
    {
        // Для O_DIRECT нужна специальная обработка
        if (!is_aligned((void*)buf, DIRECT_ALIGNMENT) || 
            (count % DIRECT_ALIGNMENT) != 0 ||
            (info->position % DIRECT_ALIGNMENT) != 0)
        {
            // Используем временный выровненный буфер
            char *aligned_buf = vtpc_alloc_aligned_buffer(count);
            if (!aligned_buf)
                return -1;
            
            raw_memcpy(aligned_buf, buf, count);
            ssize_t result = vtpc_write_aligned(fd, aligned_buf, count);
            
            vtpc_free_aligned_buffer(aligned_buf);
            return result;
        }
        
        return vtpc_write_aligned(fd, buf, count);
    }
    
    // Обычная запись без O_DIRECT с использованием кэша
    size_t total = 0;
    const char *buffer = (const char*)buf;

    while (total < count) {
        off_t pos = info->position + (off_t)total;
        off_t block_num = get_block_number(pos);
        off_t offset = get_block_offset(pos);
        
        size_t in_block = BLOCK_SIZE - offset;
        size_t needed = count - total;
        size_t copy = (in_block < needed) ? in_block : needed;
        
        spin_lock(&cache_manager.lock);
        
        CacheBlock *block = find_block_in_cache(fd, block_num);
        
        if (!block) {
            block = allocate_new_block(fd, block_num);
            if (!block) {
                spin_unlock(&cache_manager.lock);
                info->position += (off_t)total;
                return total;
            }
        }
        
        raw_memcpy(block->data + offset, buffer + total, copy);
        block->dirty = 1;
        block->last_access = raw_time();
        
        total += copy;
        
        spin_unlock(&cache_manager.lock);
        
        off_t new_end_pos = pos + (off_t)copy;
        if (new_end_pos > info->file_size) {
            info->file_size = new_end_pos;
        }
    }

    info->position += (off_t)total;
    return total;
}

/**
 * @brief Запись данных в файл с выровненным буфером
 *
 * Записывает указанное количество байт из выровненного буфера в файл.
 * Используется для прямого доступа к диску.
 *
 * @param fd Виртуальный дескриптор файла
 * @param aligned_buf Выровненный буфер с данными для записи
 * @param count Количество байт для записи
 * @return Количество записанных байт или -1 в случае ошибки
 */
ssize_t vtpc_write_aligned(int fd, const void *aligned_buf, size_t count)
{
    if (fd < 0 || fd >= 1024 || !open_files[fd] || !aligned_buf)
        return -1;
    if (!is_aligned((void *)aligned_buf, DIRECT_ALIGNMENT))
        return -1;

    FileInfo *info = open_files[fd];

    // Для прямого доступа пишем напрямую на диск
    if (info->use_direct_io)
    {
        size_t aligned_count = ((count + DIRECT_ALIGNMENT - 1) / DIRECT_ALIGNMENT) * DIRECT_ALIGNMENT;

        ssize_t bytes_written = direct_disk_write(info->fd, aligned_buf, aligned_count, info->position);

        if (bytes_written > 0)
        {
            if ((size_t)bytes_written > count)
            {
                bytes_written = count;
            }
            info->position += bytes_written;

            off_t new_end_pos = info->position;
            if (new_end_pos > info->file_size)
            {
                info->file_size = new_end_pos;
            }
        }

        return bytes_written;
    }

    // Для обычных файлов используем стандартную запись через кэш
    return vtpc_write(fd, aligned_buf, count);
}

/**
 * @brief Позиционирование в файле
 *
 * Устанавливает позицию в файле согласно указанному смещению и режиму.
 *
 * @param fd Виртуальный дескриптор файла
 * @param offset Смещение относительно точки отсчета
 * @param whence Точка отсчета (SEEK_SET, SEEK_CUR, SEEK_END)
 * @return Новая позиция в файле или -1 в случае ошибки
 */
off_t vtpc_lseek(int fd, off_t offset, int whence)
{
    if (fd < 0 || fd >= 1024 || !open_files[fd])
        return -1;

    FileInfo *info = open_files[fd];
    off_t new_position;

    switch (whence)
    {
    case SEEK_SET:
        new_position = offset;
        break;

    case SEEK_CUR:
        new_position = info->position + offset;
        break;

    case SEEK_END:
        new_position = info->file_size + offset;
        break;

    default:
        return -1;
    }

    if (new_position < 0)
    {
        return -1;
    }

    info->position = new_position;
    return new_position;
}

/**
 * @brief Синхронизация данных файла с диском
 *
 * Принудительно записывает все буферизованные данные файла на диск.
 *
 * @param fd Виртуальный дескриптор файла
 * @return 0 в случае успеха, -1 в случае ошибки
 */
int vtpc_fsync(int fd)
{
    if (fd < 0 || fd >= 1024 || !open_files[fd])
        return -1;

    FileInfo *info = open_files[fd];

    spin_lock(&cache_manager.lock);

    // Сбрасываем "грязные" блоки нашего кэша для этого файла
    for (int i = 0; i < cache_manager.capacity; i++) {
        CacheBlock *block = &cache_manager.blocks[i];
        if (block->valid && block->fd == fd && block->dirty)
        {
            off_t pos = block->block_number * BLOCK_SIZE;
            direct_disk_write(info->fd, block->data, BLOCK_SIZE, pos);
            block->dirty = 0;
        }
    }

    spin_unlock(&cache_manager.lock);

    // Вызываем системный fsync для гарантии записи на диск
    int result = raw_fsync(info->fd);

    return result;
}

// ========== ФУНКЦИИ ДЛЯ РАБОТЫ С ПРЯМЫМ ДОСТУПОМ ==========

/**
 * @brief Создание выровненного буфера для прямого доступа
 *
 * Создает буфер памяти, выровненный для прямого доступа к диску.
 *
 * @param size Размер буфера
 * @return Указатель на выровненный буфер или NULL в случае ошибки
 */
void *vtpc_alloc_aligned_buffer(size_t size)
{
    DirectIOBuffer buffer = create_direct_io_buffer(size);
    return buffer.buffer;
}

/**
 * @brief Освобождение выровненного буфера
 *
 * Освобождает память, выделенную для выровненного буфера.
 *
 * @param buffer Указатель на выровненный буфер
 */
void vtpc_free_aligned_buffer(void *buffer)
{
    if (buffer)
    {
        DirectIOBuffer db = {buffer, NULL, 0};
        free_direct_io_buffer(&db);
    }
}

/**
 * @brief Принудительная запись на диск в обход кэша
 *
 * Выполняет запись данных в файл напрямую на диск, обходя кэш,
 * даже если файл открыт без O_DIRECT.
 *
 * @param fd Виртуальный дескриптор файла
 * @param aligned_buf Выровненный буфер с данными для записи
 * @param count Количество байт для записи
 * @return Количество записанных байт или -1 в случае ошибки
 */
ssize_t vtpc_write_direct(int fd, const void *aligned_buf, size_t count)
{
    if (fd < 0 || fd >= 1024 || !open_files[fd] || !aligned_buf)
        return -1;
    if (!is_aligned((void *)aligned_buf, DIRECT_ALIGNMENT))
        return -1;

    FileInfo *info = open_files[fd];

    ssize_t bytes_written = direct_disk_write(info->fd, aligned_buf, count, info->position);

    if (bytes_written > 0)
    {
        info->position += bytes_written;
        if (info->position > info->file_size)
        {
            info->file_size = info->position;
        }
    }

    return bytes_written;
}

/**
 * @brief Очистка ресурсов
 *
 * Записывает все "грязные" блоки на диск и освобождает все ресурсы,
 * связанные с файловой системой.
 */
void vtpc_cleanup(void)
{
    spin_lock(&cache_manager.lock);

    // Записываем все "грязные" блоки на диск
    for (int i = 0; i < cache_manager.capacity; i++) {
        CacheBlock *block = &cache_manager.blocks[i];
        if (block->valid && block->dirty)
        {
            FileInfo *file_info = open_files[block->fd];
            if (file_info)
            {
                if (file_info->use_direct_io)
                {
                    write_block_to_disk_direct(block);
                }
                else
                {
                    off_t pos = block->block_number * BLOCK_SIZE;
                    off_t old_pos = raw_lseek(file_info->fd, 0, SEEK_CUR);
                    if (old_pos >= 0 && raw_lseek(file_info->fd, pos, SEEK_SET) >= 0)
                    {
                        raw_write(file_info->fd, block->data, BLOCK_SIZE);
                        raw_lseek(file_info->fd, old_pos, SEEK_SET);
                    }
                }
            }
        }
        
        // Освобождаем память блока
        if (block->valid && block->original_ptr)
        {
            raw_munmap(block->original_ptr, BLOCK_SIZE + DIRECT_ALIGNMENT - 1);
            block->data = NULL;
            block->original_ptr = NULL;
        }
        
        // Помечаем как невалидный
        block->valid = 0;
    }

    // Освобождаем массив менеджера
    if (cache_manager.blocks) {
        raw_munmap(cache_manager.blocks, sizeof(CacheBlock) * cache_manager.capacity);
        cache_manager.blocks = NULL;
    }
    
    cache_manager.block_count = 0;
    cache_manager.lru_head = -1;
    cache_manager.lru_tail = -1;

    spin_unlock(&cache_manager.lock);

    // Закрываем все открытые файлы
    for (int i = 0; i < 1024; i++)
    {
        if (open_files[i] != NULL)
        {
            FileInfo *info = open_files[i];
            raw_close(info->fd);
            raw_munmap(info, sizeof(FileInfo));
            open_files[i] = NULL;
        }
    }
}
