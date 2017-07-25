function outgoing_model = homogeneousTransformation( incoming_model, T)
%This function moves the link to it's correct place in the MATLAB figure
%when we first load it
outgoing_model = incoming_model;
for i = 1:length(incoming_model.vertices(:,1))
    outgoing_model.vertices(i,:) = T(1:3,1:3)*incoming_model.vertices(i,:)' + T(1:3,4);
end
end

