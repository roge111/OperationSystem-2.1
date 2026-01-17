# Makefile для MemoryLoader с VTPC
CC = gcc
CFLAGS = -Wall -Wextra -O2 -g -pthread -D_GNU_SOURCE
LDFLAGS = -pthread -lm

# Цели
TARGETS = libvtpc.so memory_loader_vtpc

all: $(TARGETS)

# Библиотека VTPC
libvtpc.so: vtpc.c vtpc.h
	$(CC) $(CFLAGS) -fPIC -shared -o $@ vtpc.c

# Загрузчик с VTPC кэшем
memory_loader_vtpc: MemoryLoader_vtpc.c libvtpc.so
	$(CC) $(CFLAGS) -o $@ MemoryLoader_vtpc.c -L. -lvtpc -Wl,-rpath,.

# Тестирование
test: memory_loader_vtpc
	@echo "=== Запуск Memory Loader с VTPC Cache ==="
	./memory_loader_vtpc

# Очистка
clean:
	rm -f $(TARGETS) *.o *.so memory_loader_vtpc

.PHONY: all clean test