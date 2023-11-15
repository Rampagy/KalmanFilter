@echo off
gcc lib/cholesky.c lib/kalman.c main.c lib/matrix.c -Ilib -o kalman.exe

call kalman.exe