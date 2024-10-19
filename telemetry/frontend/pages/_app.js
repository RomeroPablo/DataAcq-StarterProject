import { useEffect, useState } from 'react';
import Pusher from 'pusher-js';

function MyApp({ Component, pageProps }) {
  const [data, setData] = useState([]);

  useEffect(() => {
    const pusher = new Pusher(process.env.NEXT_PUBLIC_PUSHER_KEY, {
      cluster: process.env.NEXT_PUBLIC_PUSHER_CLUSTER,
    });

    const channel = pusher.subscribe('gyroscope-channel');
    channel.bind('new-data', function(newData) {
      setData(prevData => [...prevData, newData]);
    });

    return () => {
      pusher.unsubscribe('gyroscope-channel');
    };
  }, []);

  return <Component {...pageProps} data={data} />;
}

export default MyApp;
