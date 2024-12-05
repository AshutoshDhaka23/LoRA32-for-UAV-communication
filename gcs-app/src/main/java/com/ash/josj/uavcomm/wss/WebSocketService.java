package com.ash.josj.uavcomm.wss;


import jakarta.annotation.PostConstruct;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;
import org.springframework.web.socket.WebSocketSession;

@Service
public class WebSocketService {

    @Autowired
    private WebSocketSession webSocketSession;

    @PostConstruct
    public void init() {
        System.out.println("WebSocket connection established: " + webSocketSession.isOpen());
    }

}
