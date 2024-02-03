Uses [nanoprintf](https://github.com/charlesnicholson/nanoprintf) library

To reduce the size to 1696B:

```
CFLAGS += -Os
```

When specifying ```-0s``` there will be warnings because of compiler optimizations
for loops in startup file

```
... ld.exe: nucleo_l476rg_startup.o: in function `Reset_Handler'
... nucleo_l476rg_startup.c:230: undefined reference to `memset'
... nucleo_l476rg_startup.c:224: undefined reference to `memcpy'
```

To fix this following lines were added in the **Makefile**
CFLAGS += -fno-builtin
LDFLAGS += -nodefaultlibs