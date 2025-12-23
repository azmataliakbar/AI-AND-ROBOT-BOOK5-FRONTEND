// src/pages/chat/index.tsx
import React, { useState, useRef, useEffect } from 'react';
import Layout from '@theme/Layout';
import styles from './chat.module.css';

type Message = {
  id: string;
  text: string;
  sender: 'user' | 'ai';
  timestamp: Date;
};

export default function Chat(): React.ReactElement {
  const [messages, setMessages] = useState<Message[]>([
    {
      id: '1',
      text: '📚Hello! I\'m your AI assistant for the Physical AI & Humanoid Robotics Book.📚  Ask me anything about ROS 2, Gazebo, Isaac, or VLA models!',
      sender: 'ai',
      timestamp: new Date(),
    }
  ]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const handleSubmit = async (e: React.FormEvent) => {
    e.preventDefault();

    if (!inputValue.trim() || isLoading) return;

    // Add user message
    const userMessage: Message = {
      id: Date.now().toString(),
      text: inputValue,
      sender: 'user',
      timestamp: new Date()
    };

    setMessages(prev => [...prev, userMessage]);
    const currentQuestion = inputValue;
    setInputValue('');
    setIsLoading(true);

    try {
      // ✅ FIXED: Call backend API with correct schema
      const response = await fetch('http://localhost:8000/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query: currentQuestion,    // ✅ Changed from "message" to "query"
          user_id: null,             // ✅ Added required field
          chapter_id: null           // ✅ Added required field
        }),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();

      // ✅ FIXED: Use correct response field
      const aiMessage: Message = {
        id: (Date.now() + 1).toString(),
        text: data.response || 'I received your question but could not generate a response.',
        sender: 'ai',
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, aiMessage]);
    } catch (error) {
      console.error('Chat error:', error);
      const errorMessage: Message = {
        id: (Date.now() + 1).toString(),
        text: 'Sorry, I encountered an error. Please make sure the backend is running at http://localhost:8000',
        sender: 'ai',
        timestamp: new Date(),
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  return (
    <Layout title="AI Chat Assistant">
      <div className={styles.chatContainer}>
        <div className={styles.chatHeader}>
          <h1>🤖 AI Robotics Assistant 🤖</h1>
          <p>📚Ask me anything about the book content!📚</p>
        </div>

        <div className={styles.chatMessages}>
          {messages.map((message) => (
            <div
              key={message.id}
              className={`${styles.message} ${
                message.sender === 'user' ? styles.userMessage : styles.aiMessage
              }`}
            >
              <div className={styles.messageContent}>
                <div className={styles.messageText}>{message.text}</div>
                <div className={styles.messageTime}>
                  {message.timestamp.toLocaleTimeString([], { 
                    hour: '2-digit', 
                    minute: '2-digit' 
                  })}
                </div>
              </div>
            </div>
          ))}
          
          {isLoading && (
            <div className={`${styles.message} ${styles.aiMessage}`}>
              <div className={styles.typingIndicator}>
                <span></span>
                <span></span>
                <span></span>
              </div>
            </div>
          )}
          
          <div ref={messagesEndRef} />
        </div>

        <form onSubmit={handleSubmit} className={styles.chatInputForm}>
          <input
            type="text"
            value={inputValue}
            onChange={(e) => setInputValue(e.target.value)}
            placeholder="📚Ask about ROS 2, Gazebo, URDF, Isaac, VLA models..."
            className={styles.chatInput}
            disabled={isLoading}
            autoFocus
          />
          <button
            type="submit"
            className={styles.sendButton}
            disabled={isLoading || !inputValue.trim()}
          >
            {isLoading ? '⏳' : '➤'}
          </button>
        </form>
      </div>
    </Layout>
  );
}