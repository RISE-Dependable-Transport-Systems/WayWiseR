#!/bin/bash

# Helper script to generate TLS Certificates
# Usage: ./generate_tls_cert.bash [basename] [output_dir]
# Example: ./generate_tls_cert.bash server
# Example: ./generate_tls_cert.bash client1 custom_certs_dir

BASENAME=${1:-}
OUT_DIR=${2:-waywiser_certs}

if [ -z "$BASENAME" ]; then
    echo "Usage: $0 <basename> [output_dir]"
    echo "Example: $0 server"
    echo "Example: $0 client1 waywiser_certs"
    exit 1
fi

mkdir -p "$OUT_DIR"
cd "$OUT_DIR" || exit 1

echo "============================================="
echo " Generating Zenoh TLS Certificate: $BASENAME"
echo " Output Directory: $(pwd)"
echo "============================================="

# 1. Root CA (Generate only if missing)
if [[ ! -f "ca.crt" ]] || [[ ! -f "ca.key" ]]; then
    echo "-> Root CA not found. Creating new Certificate Authority (ca.crt)..."
    openssl req -x509 -nodes -days 1000 -newkey rsa:2048 \
        -keyout ca.key -out ca.crt \
        -subj "/CN=WayWiseR_Root_CA" 2>/dev/null
else
    echo "-> Found existing Root CA (ca.crt). Using it to sign new certificate."
fi

# 2. Target Certificates
echo "-> Creating keys and signed certificate ($BASENAME.crt)..."
openssl req -nodes -newkey rsa:2048 \
    -keyout "$BASENAME.key" -out "$BASENAME.csr" \
    -subj "/CN=waywiser_$BASENAME" 2>/dev/null

openssl x509 -req -in "$BASENAME.csr" \
    -CA ca.crt -CAkey ca.key -CAcreateserial \
    -out "$BASENAME.crt" -days 1000 2>/dev/null

# Clean up
rm -f *.csr ca.srl

echo "============================================="
echo " Success! Secure Certificates explicitly generated:"
echo ""
echo " - CA:     ca.crt (Public)"
echo " - TARGET: $BASENAME.crt (Public), $BASENAME.key (Private)"
echo "============================================="
