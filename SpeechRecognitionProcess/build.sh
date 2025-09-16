gcc -O2 -o speechRecognition speechRecognition.c \
  $(pkg-config --cflags --libs pocketsphinx sphinxbase) \
  -lasound