# Load the TFLite model
model_path = "best.onnx"  # or "tf.tflite"

# if ends with .tflite, load as TFLite model
if model_path.endswith(".tflite"):
    from tflite_support import metadata
    import tensorflow as tf
    import numpy as np
    import cv2
    interpreter = tf.lite.Interpreter(model_path=model_path)
    interpreter.allocate_tensors()

    # Get input and output details
    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()

    print(f"Model path: {model_path}")
    print("Input:", input_details)
    print("\n\n")
    print("Output:", output_details)
    print("\n\n")


    # ------- Extract metadata from the TFLite model -------
    # Load the model file
    with open(model_path, "rb") as f:
        model_content = f.read()

    # Create a metadata displayer
    displayer = metadata.MetadataDisplayer.with_model_file(model_path)

    # Get model metadata in JSON format
    model_metadata = displayer.get_metadata_json()

    # Print metadata
    print("TFLite Model Metadata:")
    print(model_metadata)

# if ends with .onnx, load as ONNX model
elif model_path.endswith(".onnx"):
    import onnxruntime as ort
    import numpy as np
    import cv2

    session = ort.InferenceSession(model_path, providers=["CPUExecutionProvider"])
    input_name = session.get_inputs()[0].name
    output_names = [o.name for o in session.get_outputs()]
    print("Input name:", input_name)
    print("Output names:", output_names)

    # Dummy input for shape info
    dummy = np.random.rand(1, 3, 320, 320).astype(np.float32)
    outputs = session.run(output_names, {input_name: dummy})

    for i, out in enumerate(outputs):
        print(f"Output {i} shape:", np.array(out).shape)