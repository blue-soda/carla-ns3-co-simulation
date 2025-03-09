def build(bld):
    obj = bld.create_ns3_program('carla-v2x-bridge', 
                                ['core', 'network', 'internet', 'mobility', 
                                 'wifi', 'wave', 'applications', 'netanim'])
    obj.source = 'carla-v2x-bridge.cc'
    obj.lib = ['zmq']  # Add ZeroMQ library