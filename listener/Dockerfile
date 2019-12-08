FROM rust:1.39-slim AS builder

WORKDIR /code
COPY . /code
RUN cargo build --release

FROM rust:1.39-slim

COPY --from=builder /code/target/release/listener /listener

CMD ["./listener"]
