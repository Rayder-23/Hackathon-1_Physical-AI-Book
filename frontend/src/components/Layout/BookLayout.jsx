import React, { useRef } from 'react';
import Layout from '@theme/Layout';
import ChatInterface from '../../components/ChatInterface/ChatInterface';

// A custom layout component for book pages with integrated chat interface
const BookLayout = ({ children, title, description }) => {
  const bookContentRef = useRef(null);

  return (
    <Layout title={title} description={description}>
      <div className="book-layout-container">
        <main ref={bookContentRef} className="book-content">
          {children}
        </main>
        <aside className="chat-sidebar">
          <ChatInterface bookContentRef={bookContentRef} />
        </aside>
      </div>
    </Layout>
  );
};

export default BookLayout;