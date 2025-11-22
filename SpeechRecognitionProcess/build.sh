gcc speechRecognition.c -o speechRecognition \
     -I/usr/include \
     -I/usr/include/pocketsphinx \
     -I/usr/include/sphinxbase \
     -lpocketsphinx -lsphinxbase -lsphinxad -lasound