if [ -f "$WAYWISER_WS/src/WayWiseR/.env" ]; then
    while IFS='=' read -r key value
    do
        # Skip comments and empty lines
        [[ "$key" =~ ^#.*$ || -z "$key" ]] && continue
        export "$key"="$value"
    done < "$WAYWISER_WS/src/WayWiseR/.env"
fi
