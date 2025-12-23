const chatbotConfig = {
  backendUrl: 'http://127.0.0.1:8000',  // Base URL without endpoint paths
  endpoints: {
    ingest: '/api/v1/ingest/',          // Trailing slash added
    query: '/api/v1/query/',            // Full path with trailing slash
    querySelection: '/api/v1/query/selection/'  // Full path with trailing slash
  },
  maxRetries: 3,
  timeout: 10000  // 10 seconds
};

export default chatbotConfig;