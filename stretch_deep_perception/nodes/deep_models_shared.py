#!/usr/bin/env python3

def print_model_info(model, text_name):

    layer_names = model.getLayerNames()
    print('{0}.getLayerNames() ='.format(text_name), layer_names)
    output_layer_indices = model.getUnconnectedOutLayers()
    print('{0}.getUnconnectedOutLayers() ='.format(text_name), output_layer_indices)
    output_layer_names = model.getUnconnectedOutLayersNames()
    print('{0} output layer names ='.format(text_name), output_layer_names)

    # "Each net always has special own the network input pseudo layer
    # with id=0. This layer stores the user blobs only and don't make
    # any computations. In fact, this layer provides the only way to
    # pass user data into the network. As any other layer, this layer
    # can label its outputs and this function provides an easy way to
    # do this." -
    # https://docs.opencv.org/4.1.2/db/d30/classcv_1_1dnn_1_1Net.html#a5e74adacffd6aa53d56046581de7fcbd
    input_layer = model.getLayer(0)
    print('{0} input layer ='.format(text_name), input_layer)
    input_layer_name = layer_names[0]
    print('{0} input layer name ='.format(text_name), input_layer_name)
        
    for layer_name in output_layer_names:
        out_layer = model.getLayer(layer_name)
        print('{0} out_layer ='.format(text_name), out_layer)
