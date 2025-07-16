if [ -f "$WAYWISER_DOTENV_PATH" ]; then
    while IFS='=' read -r key value
    do
        # Skip comments and empty lines
        [[ "$key" =~ ^#.*$ || -z "$key" ]] && continue
        export "$key"="$value"
    done < "$WAYWISER_DOTENV_PATH"
else
    echo "Warning: .env file not found at $WAYWISER_DOTENV_PATH"
fi
