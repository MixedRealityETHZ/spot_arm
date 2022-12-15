function filtered_timestamps = extract_timestamp_section(timestamps, ...
        min, max)
    filtered_timestamps = timestamps(timestamps >= min & timestamps <= max);
end