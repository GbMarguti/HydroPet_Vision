// Modelo digital da placa de interface do HydraPet para Wokwi.
// PCF8574 (endereço do projeto 0x20), divisor do ECHO e chave dos LEDs IR.
// O chip resolve níveis lógicos; não calcula tensões, correntes ou potência.
#ifndef ARDUINO
#include "wokwi-api.h"
#include <stdlib.h>

typedef struct {
  pin_t port[8], address[3];
  pin_t echo_in, echo_out, gate, drain;
  uint8_t latch;
} interface_t;

static bool connect_i2c(void *user, uint32_t address, bool read) {
  interface_t *io = user;
  (void)read;
  return address == 0x20 && !pin_read(io->address[0]) &&
         !pin_read(io->address[1]) && !pin_read(io->address[2]);
}

static uint8_t read_port(void *user) {
  interface_t *io = user;
  uint8_t value = 0;
  for (unsigned bit = 0; bit < 8; bit++) {
    if ((io->latch & (1u << bit)) && pin_read(io->port[bit])) value |= 1u << bit;
  }
  return value;
}

static bool write_port(void *user, uint8_t value) {
  interface_t *io = user;
  const uint8_t changed = io->latch ^ value;
  io->latch = value;
  for (unsigned bit = 0; bit < 8; bit++) {
    const unsigned mask = 1u << bit;
    if (!(changed & mask)) continue;
    // P3 e P4 são dedicados a saídas nesta placa. Um nível lógico alto
    // ideal evita ambiguidades entre entradas digitais de peças distintas.
    // Nos demais pinos, escrever 1 libera a linha para o contato externo.
    uint32_t high_mode = (bit == 3 || bit == 4) ? OUTPUT_HIGH : INPUT_PULLUP;
    pin_mode(io->port[bit], value & mask ? high_mode : OUTPUT_LOW);
  }
  return true;
}

static void interface_edge(void *user, pin_t pin, uint32_t level) {
  interface_t *io = user;
  if (pin == io->echo_in) pin_write(io->echo_out, level);
  if (pin == io->gate) pin_mode(io->drain, level ? OUTPUT_LOW : INPUT);
}

void chip_init(void) {
  interface_t *io = calloc(1, sizeof(*io));
  const char *ports[] = {"P0","P1","P2","P3","P4","P5","P6","P7"};
  const char *address[] = {"A0","A1","A2"};
  io->latch = 0xff;
  for (unsigned bit = 0; bit < 8; bit++) io->port[bit] = pin_init(ports[bit], INPUT_PULLUP);
  for (unsigned bit = 0; bit < 3; bit++) io->address[bit] = pin_init(address[bit], INPUT_PULLDOWN);
  io->echo_in = pin_init("ECHO5", INPUT);
  io->echo_out = pin_init("ECHO3", OUTPUT_LOW);
  io->gate = pin_init("GATE", INPUT);
  io->drain = pin_init("DRAIN", INPUT);
  pin_watch_config_t edges = { .edge = BOTH, .pin_change = interface_edge, .user_data = io };
  pin_watch(io->echo_in, &edges);
  pin_watch(io->gate, &edges);
  interface_edge(io, io->echo_in, pin_read(io->echo_in));
  interface_edge(io, io->gate, pin_read(io->gate));
  i2c_config_t bus = {
    .address = 0x20, .scl = pin_init("SCL", INPUT), .sda = pin_init("SDA", INPUT),
    .connect = connect_i2c, .read = read_port, .write = write_port, .user_data = io
  };
  i2c_init(&bus);
}
#endif
