package com.ash.josj.uavcomm.wss;

import org.springframework.beans.factory.annotation.Value;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;
import org.springframework.web.socket.WebSocketHandler;
import org.springframework.web.socket.WebSocketSession;
import org.springframework.web.socket.client.WebSocketClient;
import org.springframework.web.socket.client.standard.StandardWebSocketClient;
import org.springframework.web.socket.handler.TextWebSocketHandler;

@Configuration
public class WebSocketConfig {

    @Value("${secret.token}")
    private String accessToken;

    @Bean
    public WebSocketClient webSocketClient() {
        return new StandardWebSocketClient();
    }

    @Bean
    public WebSocketHandler webSocketHandler() {
        return new TextWebSocketHandler() {
            @Override
            public void handleTextMessage(WebSocketSession session, org.springframework.web.socket.TextMessage message) {
                // Handle incoming WebSocket messages
                System.out.println("Received: " + message.getPayload());
            }
        };
    }


    @Bean
    public WebSocketSession webSocketSession(WebSocketClient webSocketClient, WebSocketHandler webSocketHandler) throws Exception {
        String url = "wss://iotnet.teracom.dk/app?token=" + accessToken;
        return webSocketClient.doHandshake(webSocketHandler, url).get();
    }
}
