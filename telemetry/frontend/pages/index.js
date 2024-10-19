import DataChart from '../components/DataChart';

const Home = ({ data }) => {
  return (
    <div>
      <h1>Gyroscopic Data Dashboard</h1>
      <DataChart data={data} />
    </div>
  );
};

export default Home;
