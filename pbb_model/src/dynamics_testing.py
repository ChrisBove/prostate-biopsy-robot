
import dynamic_model

needle = dynamic_model.DynamicModel()

(s_state, q_state) =  needle.update(5,5,.1)
print s_state
print  "---------------------"
print q_state
(s_state, q_state) =  needle.update(5,5,.1)
print s_state
print "----------------------"
print q_state
