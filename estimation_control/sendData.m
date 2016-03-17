data = sin(1:64);
plot(data);
%Create a client interface and open it.
t = tcpip('10.20.117.254', 30000, 'NetworkRole', 'client');
fopen(t)
%Write the waveform to the server session.
fwrite(t, data)