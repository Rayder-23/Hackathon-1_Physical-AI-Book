import React, { useRef } from 'react';
import OriginalDocPage from '@theme-original/DocPage';
import ChatInterface from '@site/src/components/ChatInterface/ChatInterface';
import useDoc from '@docusaurus/useDoc';

// Custom DocPage wrapper that adds the chat interface to documentation pages
export default function DocPage(props) {
  const bookContentRef = useRef(null);
  const { metadata } = useDoc();

  return (
    <>
      <div className="book-layout-container">
        <main ref={bookContentRef} className="book-content">
          <OriginalDocPage {...props} />
        </main>
        <aside className="chat-sidebar">
          <div className="chat-interface-container">
            <h4>{metadata.title || 'Physical AI Book Assistant'}</h4>
            <ChatInterface bookContentRef={bookContentRef} />
          </div>
        </aside>
      </div>
    </>
  );
}