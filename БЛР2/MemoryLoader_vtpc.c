// MemoryLoader_vtpc.c
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <pthread.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/time.h>
#include <linux/fadvise.h>
#include <ctype.h>
#include <sys/types.h>
#include <errno.h>

#define ARRAY_SIZE 10000
#define MAX_NUM_LENGTH 150
#define BLOCK_SIZE 4096  // 1 МБ

// Добавляем заголовок нашего кэша
#include "vtpc.h"

// Структуры для мониторинга
typedef struct {
    double user_avg;
    double system_avg;
    double wait_avg;
    unsigned long context_switches_total;
    unsigned long context_switches_delta;
} monitoring_result_t;

typedef struct {
    volatile int monitoring;
    int max_processes;
} process_monitor_t;

typedef struct {
    clock_t time_total; // Время чтения
    int found_number; // Найдено ли число
} ReadData;

// Прототипы функций мониторинга (ранее были в CpuLoader.c)
void* process_monitor_thread(void* arg);
void* cpu_monitoring(void* arg);

// Вспомогательная функция для получения статистики CPU
typedef struct {
    double user;
    double system;
    double wait;
    unsigned long context_switches;
} cpu_stats_t;

cpu_stats_t get_cpu_stats_from_proc() {
    cpu_stats_t stats = {-1.0, -1.0, -1.0, 0};
    
    FILE* stat = fopen("/proc/stat", "r");
    if (!stat) {
        return stats;
    }
    
    char buffer[256];
    
    if (fgets(buffer, sizeof(buffer), stat)) {
        unsigned long user, nice, system, idle, iowait, irq, softirq;
        if (sscanf(buffer, "cpu %lu %lu %lu %lu %lu %lu %lu", 
                  &user, &nice, &system, &idle, &iowait, &irq, &softirq) >= 4) {
            
            unsigned long total = user + nice + system + idle + iowait + irq + softirq;
            if (total > 0) {
                stats.user = (user + nice) * 100.0 / total;
                stats.system = system * 100.0 / total;
                stats.wait = iowait * 100.0 / total;
            }
        }
    }
    
    while (fgets(buffer, sizeof(buffer), stat)) {
        if (strncmp(buffer, "ctxt", 4) == 0) {
            unsigned long ctxt;
            if (sscanf(buffer, "ctxt %lu", &ctxt) == 1) {
                stats.context_switches = ctxt;
                break;
            }
        }
    }
    
    fclose(stat);
    return stats;
}

// Функция мониторинга CPU
void* cpu_monitoring(void* arg) {
    monitoring_result_t* result = malloc(sizeof(monitoring_result_t));
    if (!result) return NULL;

    double total_user = 0.0, total_wait = 0.0, total_sys = 0.0;
    int valid_samples = 0;
    unsigned long first_context_switches = 0;
    unsigned long last_context_switches = 0;
    
    for (int i = 0; i < 5; i++) {
        cpu_stats_t monitor = get_cpu_stats_from_proc();
        
        if (monitor.user >= 0 && monitor.system >= 0 && monitor.wait >= 0) {
            total_user += monitor.user;
            total_wait += monitor.wait;
            total_sys += monitor.system;
            valid_samples++;
            
            if (i == 0) {
                first_context_switches = monitor.context_switches;
            }
            last_context_switches = monitor.context_switches;
        }
        usleep(500000);
    }

    if (valid_samples > 0) {
        result->system_avg = total_sys / valid_samples;
        result->user_avg = total_user / valid_samples;
        result->wait_avg = total_wait / valid_samples;
        result->context_switches_total = last_context_switches;
        result->context_switches_delta = last_context_switches - first_context_switches;
    } else {
        result->system_avg = result->user_avg = result->wait_avg = -1.0;
        result->context_switches_total = result->context_switches_delta = 0;
    }

    return result;
}

// Функция мониторинга процессов
void* process_monitor_thread(void* arg) {
    process_monitor_t* monitor = (process_monitor_t*)arg;
    FILE *fp;
    char buffer[128];
    int loader_pid;
    int total_count = 0;
    int exclude_count = 0;

    fp = popen("ps -e -o pid= | wc -l", "r");
    if (!fp) {
        monitor->max_processes = 0;
        return NULL;
    }
    if (fscanf(fp, "%d", &total_count) != 1) {
        pclose(fp);
        monitor->max_processes = 0;
        return NULL;
    }
    pclose(fp);

    fp = popen("pgrep -f '^\\.\\/loaders$'", "r");
    if (!fp) {
        monitor->max_processes = total_count;
        return NULL;
    }

    while (fgets(buffer, sizeof(buffer), fp)) {
        if (sscanf(buffer, "%d", &loader_pid) == 1) {
            exclude_count++;
        }
    }
    pclose(fp);

    monitor->max_processes = exclude_count;
    return NULL;
}

// Функция случайного чтения с использованием VTPC кэша
// Функция случайного чтения с использованием VTPC кэша + временные файлы
ReadData randomReadTest_vtpc(int number) {
    ReadData result = { .time_total = 0, .found_number = 0 };

    char number_str[MAX_NUM_LENGTH];
    snprintf(number_str, sizeof(number_str), "%d", number);
    
    // Открываем файл через наш кэш
    int fd = vtpc_open("fragments_numbers_2.txt", VTPC_O_RDONLY | VTPC_O_DIRECT);
    if (fd < 0) {
        printf("Error: failed to open file with vtpc_open\n");
        return result;
    }
    
    // Получаем размер файла
    off_t file_size = vtpc_lseek(fd, 0, SEEK_END);
    if (file_size <= 0) {
        printf("Error: failed to get file size\n");
        vtpc_close(fd);
        return result;
    }
    
    // Возвращаемся в начало
    vtpc_lseek(fd, 0, SEEK_SET);
    
    // Выделяем буфер
    char* buffer = malloc(BLOCK_SIZE);
    if (!buffer) {
        printf("Error: failed to allocate memory\n");
        vtpc_close(fd);
        return result;
    }
    
    // Проверяем размер файла
    if (file_size <= BLOCK_SIZE) {
        printf("Error: file size is too small\n");
        free(buffer);
        vtpc_close(fd);
        return result;
    }

    // Генерация случайного смещения
    srand((unsigned int)time(NULL) + getpid());
    off_t max_offset = file_size - BLOCK_SIZE;
    off_t offset = (off_t)(rand() % (int)(max_offset / 4096)) * 4096; // Выравнивание по 4KB
    
    // Перемещаемся на случайную позицию
    vtpc_lseek(fd, offset, SEEK_SET);

    // Замер времени чтения через наш кэш
    clock_t start_read = clock();
    
    // Чтение блока данных через наш кэш
    ssize_t bytesRead = vtpc_read(fd, buffer, BLOCK_SIZE);
    
    clock_t end_read = clock();
    
    // Проверка успешности чтения
    if (bytesRead != BLOCK_SIZE) {
        printf("Warning: read %zd bytes, expected %d\n", bytesRead, BLOCK_SIZE);
    }
    
    // Поиск числа в блоке
    size_t num_len = strlen(number_str);
    if (num_len == 0 || num_len > 150) {
        free(buffer);
        vtpc_close(fd);
        return result;
    }
    
    clock_t start_replace = 0;
    clock_t end_replace = 0;
    int replaced = 0;
    
    // Простой поиск числа (упрощенный вариант)
    for (size_t i = 0; i < BLOCK_SIZE - num_len; i++) {
        if (memcmp(buffer + i, number_str, num_len) == 0) {
            // Проверка границ
            int left_ok = (i == 0) || !isalnum(buffer[i - 1]);
            int right_ok = (i + num_len >= BLOCK_SIZE) || !isalnum(buffer[i + num_len]);
            
            if (left_ok && right_ok) {
                start_replace = clock();
                memcpy(buffer + i, number_str, num_len);
                end_replace = clock();
                result.found_number = 1;
                replaced = 1;
                break;
            }
        }
    }
    
    // Записываем измененный блок через наш кэш, если число было найдено
    clock_t start_write = 0;
    clock_t end_write = 0;
    
    if (result.found_number == 1) {
        start_write = clock();
        vtpc_lseek(fd, offset, SEEK_SET);
        ssize_t written = vtpc_write(fd, buffer, BLOCK_SIZE);
        vtpc_fsync(fd);  // Принудительная синхронизация
        end_write = clock();
        
        if (written != BLOCK_SIZE) {
            printf("Warning: wrote %zd bytes, expected %d\n", written, BLOCK_SIZE);
        }
    }
    
    // ВАЖНО: ДОБАВЛЯЕМ ИНТЕНСИВНУЮ ЗАПИСЬ ВО ВРЕМЕННЫЕ ФАЙЛЫ
    clock_t start_io_write = clock();
    
    // Создаем 10 временных файлов и пишем в них через VTPC
    for (int j = 0; j < 10; j++) {
        char write_filename[256];
        sprintf(write_filename, "temp_io_load_vtpc_%d_%d.tmp", getpid(), rand());
        
        // Открываем временный файл через VTPC
        int temp_fd = vtpc_open(write_filename, VTPC_O_RDWR | VTPC_O_CREAT);
        if (temp_fd >= 0) {
            // Пишем несколько раз в тот же файл
            for (int k = 0; k < 5; k++) {
                vtpc_lseek(temp_fd, 0, SEEK_SET);
                vtpc_write(temp_fd, buffer, BLOCK_SIZE);
                vtpc_fsync(temp_fd);  // Принудительная синхронизация через VTPC
            }
            vtpc_close(temp_fd);
            
            // Удаляем временный файл (обычным unlink, т.к. VTPC файл уже закрыт)
            unlink(write_filename);
        }
    }
    
    clock_t end_io_write = clock();
    
    // Закрываем основной файл
    vtpc_close(fd);
    
    // Вычисляем общее время
    result.time_total = (end_io_write - start_io_write); // Используем время интенсивной записи
    
    free(buffer);
    
    return result;
}

// И аналогично нужно обновить randomReadTest_system:
ReadData randomReadTest_system(int number) {
    ReadData result = { .time_total = 0, .found_number = 0 };

    char number_str[MAX_NUM_LENGTH];
    snprintf(number_str, sizeof(number_str), "%d", number);
    
    // Для O_DIRECT требуется выравнивание
    const size_t alignment = 512;
    const size_t aligned_block_size = ((BLOCK_SIZE + alignment - 1) / alignment) * alignment;
    
    #ifndef O_DIRECT
    #define O_DIRECT 040000
    #endif
    
    // Открываем файл с прямым доступом
    int fd = open("fragments_numbers_2.txt", O_RDONLY | O_DIRECT);
    if (fd == -1) {
        fd = open("fragments_numbers_2.txt", O_RDONLY);
        if (fd == -1) {
            printf("Error: failed to open file\n");
            return result;
        }
    }
    
    // Получаем размер файла
    off_t file_size = lseek(fd, 0, SEEK_END);
    if (file_size == -1) {
        printf("Error: failed to get file size\n");
        close(fd);
        return result;
    }
    
    lseek(fd, 0, SEEK_SET);
    
    // Выделяем выровненный буфер
    char* buffer = NULL;
    if (posix_memalign((void**)&buffer, alignment, aligned_block_size) != 0) {
        printf("Error: failed to allocate aligned memory\n");
        close(fd);
        return result;
    }
    
    if (file_size <= (off_t)aligned_block_size) {
        printf("Error: file size is too small\n");
        free(buffer);
        close(fd);
        return result;
    }

    // Генерация случайного смещения
    srand((unsigned int)time(NULL) + getpid());
    off_t max_offset = file_size - aligned_block_size;
    off_t offset = (rand() % ((int)(max_offset / alignment))) * alignment;
    
    // Перемещаемся на случайную позицию
    lseek(fd, offset, SEEK_SET);

    // Замер времени
    clock_t start_read = clock();
    
    // Чтение блока данных
    ssize_t bytesRead = read(fd, buffer, aligned_block_size);
    
    // Отключаем кэширование после чтения
    posix_fadvise(fd, offset, aligned_block_size, POSIX_FADV_DONTNEED);

    clock_t end_read = clock();
    
    if (bytesRead != (ssize_t)aligned_block_size) {
        printf("Error: failed to read file, read %zd bytes\n", bytesRead);
        free(buffer);
        close(fd);
        return result;
    }

    // Поиск и замена числа
    size_t num_len = strlen(number_str);
    if (num_len == 0 || num_len > 150) {
        free(buffer);
        close(fd);
        return result;
    }
    
    clock_t start_replace = 0;
    clock_t end_replace = 0;
    int replaced = 0;
    
    for (size_t i = 0; i < aligned_block_size - num_len; i++) {
        if (memcmp(buffer + i, number_str, num_len) == 0) {
            int left_ok = (i == 0) || !isalnum(buffer[i - 1]);
            int right_ok = (i + num_len >= aligned_block_size) || !isalnum(buffer[i + num_len]);
            
            if (left_ok && right_ok) {
                start_replace = clock();
                memcpy(buffer + i, number_str, num_len);
                end_replace = clock();
                result.found_number = 1;
                replaced = 1;
                break;
            }
        }
    }
    
    clock_t start_write = 0;
    clock_t end_write = 0;
    
    if (result.found_number == 1) {
        start_write = clock();
        int write_fd = open("modified_block_system.txt", O_WRONLY | O_CREAT | O_TRUNC, 0644);
        if (write_fd != -1) {
            write(write_fd, buffer, bytesRead);
            close(write_fd);
        }
        end_write = clock();
    }
    
    // ВАЖНО: ДОБАВЛЯЕМ ИНТЕНСИВНУЮ ЗАПИСЬ ВО ВРЕМЕННЫЕ ФАЙЛЫ
    clock_t start_io_write = clock();
    
    // Создаем 10 временных файлов и пишем в них
    for (int j = 0; j < 10; j++) {
        char write_filename[256];
        sprintf(write_filename, "temp_io_load_system_%d_%d.tmp", getpid(), rand());
        
        int write_fd = open(write_filename, O_WRONLY | O_CREAT | O_DIRECT | O_SYNC, 0644);
        if (write_fd != -1) {
            // Пишем несколько раз в тот же файл
            for (int k = 0; k < 5; k++) {
                lseek(write_fd, 0, SEEK_SET);
                write(write_fd, buffer, bytesRead);
                fsync(write_fd);  // Принудительная синхронизация
            }
            close(write_fd);
            unlink(write_filename);  // Удаляем сразу
        }
    }
    
    clock_t end_io_write = clock();
    
    // Вычисляем общее время
    result.time_total = (end_io_write - start_io_write); // Используем время интенсивной записи
    
    // Освобождаем ресурсы
    free(buffer);
    close(fd);
    
    return result;
}

// Старая версия для сравнения (без кэша)


// Основная функция нагрузки на память с использованием VTPC кэша
clock_t memory_loader_vtpc(int number, double* user_avg, double* system_avg, double* wait_avg,
                          unsigned long* context_switches_total, unsigned long* context_switches_delta, 
                          int* parallel_processes)
{
    // Инициализация выходных параметров
    *user_avg = 0.0;
    *system_avg = 0.0;
    *wait_avg = 0.0;
    *context_switches_total = 0;
    *context_switches_delta = 0;
    *parallel_processes = 0;

    // Мониторинг во время чтения
    pthread_t monitoring_thread_read;
    monitoring_result_t* result_read = NULL;

    process_monitor_t process_monitor = {
        .monitoring = 1,
        .max_processes = 0
    };
    pthread_t process_monitor_thread_id;
    
    clock_t total_time_read = 0;
    
    if (pthread_create(&monitoring_thread_read, NULL, cpu_monitoring, NULL) == 0) {
        pthread_create(&process_monitor_thread_id, NULL, process_monitor_thread, &process_monitor);
        
        // Выполняем 100 операций чтения через наш кэш
        for (int i = 0; i < 100; i++) {
            ReadData result = randomReadTest_vtpc(number);
            total_time_read += result.time_total;
            
            // Можно раскомментировать для отладки
            // if (result.found_number == 1) {
            //     printf("Found number at iteration %d\n", i);
            //     break;
            // }
        }

        process_monitor.monitoring = 0;
        pthread_join(process_monitor_thread_id, NULL);
        *parallel_processes = process_monitor.max_processes;

        pthread_join(monitoring_thread_read, (void**)&result_read);
    } else {
        // Без мониторинга
        for (int i = 0; i < 100; i++) {
            ReadData result = randomReadTest_vtpc(number);
            total_time_read += result.time_total;
        }
    }

    // Обработка результатов мониторинга
    if (result_read && result_read->user_avg >= 0) {
        *user_avg = result_read->user_avg;
        *system_avg = result_read->system_avg;
        *wait_avg = result_read->wait_avg;
        *context_switches_total = result_read->context_switches_total;
        *context_switches_delta = result_read->context_switches_delta;
        free(result_read);
    } else {
        *user_avg = *system_avg = *wait_avg = -1.0;
        *context_switches_total = 0;
        *context_switches_delta = 0;
    }

    return total_time_read;
}

// Основная функция нагрузки на память с системными вызовами (для сравнения)
clock_t memory_loader_system(int number, double* user_avg, double* system_avg, double* wait_avg,
                            unsigned long* context_switches_total, unsigned long* context_switches_delta, 
                            int* parallel_processes)
{
    *user_avg = 0.0;
    *system_avg = 0.0;
    *wait_avg = 0.0;
    *context_switches_total = 0;
    *context_switches_delta = 0;
    *parallel_processes = 0;

    pthread_t monitoring_thread_read;
    monitoring_result_t* result_read = NULL;

    process_monitor_t process_monitor = {
        .monitoring = 1,
        .max_processes = 0
    };
    pthread_t process_monitor_thread_id;
    
    clock_t total_time_read = 0;
    
    if (pthread_create(&monitoring_thread_read, NULL, cpu_monitoring, NULL) == 0) {
        pthread_create(&process_monitor_thread_id, NULL, process_monitor_thread, &process_monitor);
        
        for (int i = 0; i < 100; i++) {
            ReadData result = randomReadTest_system(number);
            total_time_read += result.time_total;
        }

        process_monitor.monitoring = 0;
        pthread_join(process_monitor_thread_id, NULL);
        *parallel_processes = process_monitor.max_processes;

        pthread_join(monitoring_thread_read, (void**)&result_read);
    } else {
        for (int i = 0; i < 100; i++) {
            ReadData result = randomReadTest_system(number);
            total_time_read += result.time_total;
        }
    }

    if (result_read && result_read->user_avg >= 0) {
        *user_avg = result_read->user_avg;
        *system_avg = result_read->system_avg;
        *wait_avg = result_read->wait_avg;
        *context_switches_total = result_read->context_switches_total;
        *context_switches_delta = result_read->context_switches_delta;
        free(result_read);
    } else {
        *user_avg = *system_avg = *wait_avg = -1.0;
        *context_switches_total = 0;
        *context_switches_delta = 0;
    }

    return total_time_read;
}

// Вспомогательная функция для предотвращения оптимизации
void use_data(char* str) {
    volatile char dummy;
    while (*str) {
        dummy = *str;
        (void)dummy;   
        str++;
    }
}

// Тестовая функция для сравнения производительности
void compare_performance() {
    printf("=== Сравнение производительности VTPC кэша и системных вызовов ===\n\n");
    
    int test_number = 12345;  // Число для поиска
    
    printf("Запуск теста с VTPC кэшем...\n");
    double vtpc_user, vtpc_system, vtpc_wait;
    unsigned long vtpc_ctx_total, vtpc_ctx_delta;
    int vtpc_processes;
    
    clock_t vtpc_time = memory_loader_vtpc(test_number, &vtpc_user, &vtpc_system, &vtpc_wait,
                                          &vtpc_ctx_total, &vtpc_ctx_delta, &vtpc_processes);
    
    printf("\nЗапуск теста с системными вызовами (O_DIRECT)...\n");
    double sys_user, sys_system, sys_wait;
    unsigned long sys_ctx_total, sys_ctx_delta;
    int sys_processes;
    
    clock_t sys_time = memory_loader_system(test_number, &sys_user, &sys_system, &sys_wait,
                                           &sys_ctx_total, &sys_ctx_delta, &sys_processes);
    
    printf("\n=== РЕЗУЛЬТАТЫ СРАВНЕНИЯ ===\n");
    printf("VTPC Cache:\n");
    printf("  Время выполнения: %ld тактов (%.3f сек)\n", vtpc_time, (double)vtpc_time / CLOCKS_PER_SEC);
    printf("  Загрузка CPU: user=%.1f%%, system=%.1f%%, wait=%.1f%%\n", vtpc_user, vtpc_system, vtpc_wait);
    printf("  Переключения контекста: %lu\n", vtpc_ctx_delta);
    printf("  Процессов: %d\n", vtpc_processes);
    
    printf("\nSystem Calls (O_DIRECT):\n");
    printf("  Время выполнения: %ld тактов (%.3f сек)\n", sys_time, (double)sys_time / CLOCKS_PER_SEC);
    printf("  Загрузка CPU: user=%.1f%%, system=%.1f%%, wait=%.1f%%\n", sys_user, sys_system, sys_wait);
    printf("  Переключения контекста: %lu\n", sys_ctx_delta);
    printf("  Процессов: %d\n", sys_processes);
    
    printf("\n=== СРАВНЕНИЕ ===\n");
    if (sys_time > 0) {
        double speedup = (double)sys_time / vtpc_time;
        printf("Ускорение VTPC vs System: %.2fx\n", speedup);
        
        if (speedup > 1.0) {
            printf("✅ VTPC кэш работает БЫСТРЕЕ системных вызовов\n");
        } else if (speedup < 1.0) {
            printf("⚠️ VTPC кэш работает МЕДЛЕННЕЕ системных вызовов\n");
        } else {
            printf("⚖️ Производительность одинаковая\n");
        }
    }
}

// Простая тестовая программа
int main() {
    printf("=== Memory Loader с VTPC Cache ===\n");
    
    // Проверяем наличие тестового файла
    FILE* test_file = fopen("fragments_numbers.bin", "r");
    if (!test_file) {
        printf("Ошибка: файл fragments_numbers_2.txt не найден!\n");
        printf("Создайте тестовый файл командой:\n");
        printf("  python3 generate_fragments.py\n");
        return 1;
    }
    fclose(test_file);
    
    // Запускаем сравнение производительности
    compare_performance();
    
    // Очищаем ресурсы VTPC кэша
    // (добавьте функцию vtpc_cleanup() в ваш vtpc.c если еще нет)
    
    return 0;
}