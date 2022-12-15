function [tfs, Ts, timestamps] = get_transforms(tf_data, frame_id, child_frame_id)
    tfs = cell(0,1);
    Ts = cell(0,1);
    timestamps = [];
    for i = 1:length(tf_data)
        for j = 1:length(tf_data{i})
            tf_msg = tf_data{i}.Transforms(j);
            if strcmp(tf_msg.Header.FrameId, frame_id) && ...
                    strcmp(tf_msg.ChildFrameId, child_frame_id)
                tf = tf_msg.Transform;
                position = [tf.Translation.X, tf.Translation.Y, ...
                    tf.Translation.Z];
                quaternion = [tf.Rotation.W, tf.Rotation.X, ...
                    tf.Rotation.Y, tf.Rotation.Z];
                tfs{length(tfs)+1} = struct;
                tfs{length(tfs)}.position = position;
                tfs{length(tfs)}.quaternion = quaternion;
                Ts{length(tfs)+1} = to_transform(position, quaternion);
                timestamp = double(tf_msg.Header.Stamp.Sec) + ...
                    double(tf_msg.Header.Stamp.Nsec) / 1.0e9;
                timestamps(length(timestamps)+1) = timestamp;
            end
        end
    end
end