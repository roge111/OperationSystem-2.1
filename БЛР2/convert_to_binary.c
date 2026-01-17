// convert_to_binary.c
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define BUFFER_SIZE 4096

int main() {
    FILE *text = fopen("fragments_numbers_2.txt", "r");
    FILE *binary = fopen("fragments_numbers.bin", "wb");
    
    if (!text || !binary) {
        printf("Ошибка открытия файлов\n");
        return 1;
    }
    
    char buffer[BUFFER_SIZE];
    int number;
    int count = 0;
    
    while (fscanf(text, "%d", &number) == 1) {
        fwrite(&number, sizeof(int), 1, binary);
        count++;
        
        if (count % 10000 == 0) {
            printf("Конвертировано %d чисел...\n", count);
        }
    }
    
    fclose(text);
    fclose(binary);
    
    printf("Конвертация завершена: %d чисел\n", count);
    printf("Размер бинарного файла: %ld байт\n", (long)count * sizeof(int));
    
    return 0;
}