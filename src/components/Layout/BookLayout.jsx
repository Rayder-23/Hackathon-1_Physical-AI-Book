import React from 'react';
import Layout from '@theme/Layout';

// A custom layout component for book pages if needed
// Currently just wraps the default Docusaurus layout
const BookLayout = ({ children, title, description }) => {
  return (
    <Layout title={title} description={description}>
      <main>{children}</main>
    </Layout>
  );
};

export default BookLayout;