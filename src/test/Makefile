# コンパイラを指定
CC := g++
# インクルードファイル等
CFLAGS := `pkg-config opencv --cflags` `pkg-config opencv --libs`
LDFLAGS :=
# ディレクトリ内の全てのC++ファイルをコンパイル
SOURCES := transparent_recognition.cpp
# C++ファイルの.cppをとったものを実行ファイルの名前とする
EXECUTABLE := test

all:$(EXECUTABLE)

$(EXECUTABLE):$(SOURCES)
	$(CC) $< $(LDFLAGS) $(CFLAGS) -o $@

clean:
	rm -rf $(EXECUTABLE)
