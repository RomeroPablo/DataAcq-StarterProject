import { Line } from 'react-chartjs-2';

const DataChart = ({ data }) => {
  const chartData = {
    labels: data.map((_, index) => index.toString()),
    datasets: [
      {
        label: 'Gyroscopic Data',
        data: data.map(d => d.value),  // Adjust based on the actual structure of your data
        borderColor: 'rgba(75,192,192,1)',
        fill: false,
      },
    ],
  };

  const options = {
    scales: {
      x: { beginAtZero: true },
      y: { beginAtZero: true },
    },
  };

  return <Line data={chartData} options={options} />;
};

export default DataChart;
