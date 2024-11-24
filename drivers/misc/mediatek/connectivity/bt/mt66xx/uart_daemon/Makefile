LAUNCHER_SDIO=uart_launcher
LAUNCHER_SDIO_SRC=uart_launcher.c

all: $(LAUNCHER_SDIO)

$(LAUNCHER_SDIO):
	$(CC) -o $(LAUNCHER_SDIO) $(LAUNCHER_SDIO_SRC)

clean:
	rm $(LAUNCHER_UART) ${LAUNCHER_SDIO}
