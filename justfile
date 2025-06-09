tiramisu:
    cargo objcopy --no-default-features --release -- -O ihex
    python uf2conv.py target/thumbv7em-none-eabihf/release/tiramisu-fw-ng -o tiramisu-fw-ng.uf2 --family 0xada52840
mascarpone:
    cargo objcopy --no-default-features --release -p mascarpone -- -O ihex
    python uf2conv.py target/thumbv7em-none-eabihf/release/mascarpone -o mascarpone.uf2 --family 0xada52840
