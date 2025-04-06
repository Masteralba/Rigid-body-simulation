# Компилятор
CC = g++

CFLAGS = -g

# Библиотеки для линковки
LDFLAGS = -lGL -lGLU -lglut

# Исходные файлы
SRCS = main.cpp visual.cpp rigidbody.c help_functions.c collision.c

# Объектные файлы
OBJS = $(SRCS:.cpp=.o)
OBJS := $(OBJS:.c=.o)

# Имя исполняемого файла
TARGET = rigid_body_simulation

# Правило по умолчанию
all: $(TARGET)

# Сборка исполняемого файла
$(TARGET): $(OBJS)
	$(CC) -o $(TARGET) $(OBJS) $(LDFLAGS)

# Компиляция .cpp файлов
%.o: %.cpp
	$(CC) $(CFLAGS) -c $< -o $@

# Компиляция .c файлов
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# Очистка
clean:
	rm -f $(OBJS) $(TARGET)

# Пересборка
rebuild: clean all